//! Aggregated receive state: latest values plus running counters.
//!
//! Deliberately free of I/O and of any notion of time (except the `fps` field,
//! which the driver fills in), so that replaying a captured byte stream
//! produces exactly the same numbers as a live session.

use sbus_protocol::{Event, Frame, Telemetry};

/// Running totals over the life of a receive session.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    /// Control frames decoded.
    pub frames: u64,
    /// Telemetry slot responses decoded.
    pub slots: u64,
    /// Slot responses with no verified decode (see [`Telemetry::Unknown`]).
    pub unknown_slots: u64,
    /// Bytes discarded while resynchronising.
    ///
    /// Should be 0 on a healthy link. A nonzero value that tracks the slot
    /// rate means telemetry is being thrown away rather than parsed.
    pub desync_bytes: u64,
}

/// Latest decoded values and counters.
#[derive(Debug, Clone, Copy, Default)]
pub struct State {
    /// Most recent control frame, if any has arrived.
    pub frame: Option<Frame>,
    /// Whether any S.BUS2 footer has been seen.
    pub sbus2: bool,
    /// Receiver supply voltage ("Rx-Batt"), volts.
    pub rx_battery_v: Option<f32>,
    /// External voltage input ("Ext-Volt"), volts.
    pub external_v: Option<f32>,
    /// Running totals.
    pub counters: Counters,
    /// Control frames per second, updated by the driver over a 1 s window.
    pub fps: f32,
}

impl State {
    /// Fold one parser event into the state.
    pub fn apply(&mut self, event: &Event) {
        match event {
            Event::Frame { frame, .. } => {
                self.counters.frames += 1;
                self.sbus2 |= frame.footer.group().is_some();
                self.frame = Some(*frame);
            }
            Event::Slot { response, .. } => {
                self.counters.slots += 1;
                match response.telemetry {
                    Telemetry::RxBattery { volts, .. } => self.rx_battery_v = Some(volts),
                    Telemetry::ExternalVoltage { volts, .. } => self.external_v = Some(volts),
                    Telemetry::Unknown { .. } => self.counters.unknown_slots += 1,
                }
            }
            Event::Desync { .. } => self.counters.desync_bytes += 1,
        }
    }

    /// True once both receiver voltages have been observed.
    pub fn has_voltages(&self) -> bool {
        self.rx_battery_v.is_some() && self.external_v.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sbus_protocol::Parser;

    const FRAME_G0: [u8; 25] = [
        0x0F, 0xF9, 0x5B, 0xDF, 0x02, 0xED, 0x07, 0x04, 0x20, 0x00, 0x1F, 0xF8, 0x40, 0x00, 0x3E,
        0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x04,
    ];

    fn fold(bytes: &[u8]) -> State {
        let mut parser = Parser::new();
        let mut state = State::default();
        parser.push_slice(bytes, |e| state.apply(&e));
        state
    }

    #[test]
    fn tracks_both_voltages_and_counters() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&[0x03, 0xC0, 0x31]);
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&[0x03, 0xC4, 0xF1]);
        stream.extend_from_slice(&FRAME_G0);

        let state = fold(&stream);
        assert_eq!(state.rx_battery_v, Some(4.9));
        assert_eq!(state.external_v, Some(24.1));
        assert!(state.has_voltages());
        assert!(state.sbus2);
        assert_eq!(
            state.counters,
            Counters {
                frames: 3,
                slots: 2,
                unknown_slots: 0,
                desync_bytes: 0,
            }
        );
    }

    #[test]
    fn unknown_slots_are_counted_not_stored_as_voltage() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&[0x03, 0xC8, 0x12]);
        stream.extend_from_slice(&FRAME_G0);

        let state = fold(&stream);
        assert_eq!(state.counters.unknown_slots, 1);
        assert_eq!(state.counters.slots, 1);
        assert_eq!(state.rx_battery_v, None);
        assert_eq!(state.external_v, None);
    }

    #[test]
    fn garbage_counts_as_desync_bytes() {
        let mut stream = vec![0xAA, 0xBB];
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&FRAME_G0);

        let state = fold(&stream);
        assert_eq!(state.counters.desync_bytes, 2);
        assert_eq!(state.counters.frames, 2);
    }
}
