//! S.BUS2 telemetry slot decoding.
//!
//! Each slot response is three bytes: a slot ID followed by two data bytes.
//! Only slot 0 has a decode verified against a transmitter's telemetry
//! display; everything else is surfaced as [`Telemetry::Unknown`] rather than
//! guessed at (see `doc/spec.md` §7).

/// Length of one telemetry slot response.
pub const SLOT_LEN: usize = 3;

/// Number of telemetry slots addressable across the four groups.
pub const SLOT_COUNT: usize = 32;

/// Slots per footer group.
pub const SLOTS_PER_GROUP: usize = 8;

/// Slot 0 marker with the value's high bits masked off, selecting the
/// receiver supply voltage ("Rx-Batt").
pub const MARKER_RX_BATTERY: u8 = 0xC0;

/// Slot 0 marker with the value's high bits masked off, selecting the
/// external voltage input ("Ext-Volt").
pub const MARKER_EXTERNAL_VOLTAGE: u8 = 0xC4;

/// Marker bits that select the sensor, i.e. everything but the value's
/// high bits.
pub const MARKER_SENSOR_MASK: u8 = 0xFC;

/// Marker bits holding bits 9..8 of the voltage.
///
/// The measurement is 10 bits: bits 7..0 in the value byte, bits 9..8 in the
/// low two bits of the marker. Both verified captures below 25.6 V had these
/// bits clear, which is why an 8-bit reading fitted them (`doc/spec.md` §5.2).
pub const MARKER_VALUE_HIGH_MASK: u8 = 0x03;

/// Volts per LSB of the slot 0 voltage value.
pub const VOLT_LSB_V: f32 = 0.1;

/// Highest voltage representable by the 10-bit slot 0 value.
///
/// Covers the 0–70 V the receiver's manual gives for the external voltage
/// input. What happens above this has not been verified (`doc/spec.md` §7-1).
pub const VOLT_MAX_V: f32 = 102.3;

/// Reverse the low 6 bits of `value`.
const fn reverse_bits_6(value: u8) -> u8 {
    let v = value & 0x3F;
    let mut out = 0u8;
    let mut bit = 0;
    while bit < 6 {
        if v & (1 << bit) != 0 {
            out |= 1 << (5 - bit);
        }
        bit += 1;
    }
    out
}

/// Wire ID for telemetry slot `index` (0..=31).
///
/// The ID is the slot number's low 6 bits reversed, shifted up by two, with
/// `0b11` in the low bits. Computing it keeps the 32-entry table out of the
/// source, where a transcription slip would be invisible.
///
/// Only slot 0 (`0x03`) has been observed on hardware.
pub const fn slot_id(index: u8) -> u8 {
    (reverse_bits_6(index) << 2) | 0b11
}

/// Slot number for a wire ID, or `None` if `id` is not a valid slot ID.
///
/// Exactly 32 of the 256 byte values are valid: those matching `0b_____011`.
/// The low two bits are the fixed `0b11`; bit 2 must be clear because it
/// reverses into bit 5 of the index, which would address a 33rd..64th slot
/// that does not exist. That exclusion is what keeps [`crate::frame::START`]
/// (`0x0F`, which does end in `0b11`) out of the slot ID space — the parser
/// depends on it to tell a frame head from a slot head.
pub const fn slot_index(id: u8) -> Option<u8> {
    if id & 0b11 != 0b11 {
        return None;
    }
    let index = reverse_bits_6(id >> 2);
    if index as usize >= SLOT_COUNT {
        return None;
    }
    Some(index)
}

/// Decoded meaning of a slot response's two data bytes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Telemetry {
    /// Slot 0, marker `0xC0..=0xC3` — receiver supply voltage, "Rx-Batt".
    RxBattery {
        /// Volts.
        volts: f32,
        /// Raw 10-bit 0.1 V/LSB value.
        raw: u16,
    },
    /// Slot 0, marker `0xC4..=0xC7` — external voltage input, "Ext-Volt".
    ExternalVoltage {
        /// Volts.
        volts: f32,
        /// Raw 10-bit 0.1 V/LSB value.
        raw: u16,
    },
    /// A structurally valid slot response with no verified decode.
    ///
    /// Reached for every slot other than 0, and for slot 0 with an unexpected
    /// marker. Kept rather than dropped so callers can count and dump it —
    /// a marker change above 102.3 V would show up here.
    Unknown {
        /// The two data bytes, as received.
        data: [u8; 2],
    },
}

impl Telemetry {
    /// Volts, for the variants that carry a verified voltage.
    pub fn volts(&self) -> Option<f32> {
        match *self {
            Telemetry::RxBattery { volts, .. } | Telemetry::ExternalVoltage { volts, .. } => {
                Some(volts)
            }
            Telemetry::Unknown { .. } => None,
        }
    }
}

/// One decoded telemetry slot response.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlotResponse {
    /// Slot number 0..=31.
    pub index: u8,
    /// What the two data bytes mean.
    pub telemetry: Telemetry,
}

/// Convert a 0.1 V/LSB byte to volts.
///
/// Division by 10 rather than multiplication by `0.1_f32`: the latter is not
/// exact in binary, so 241 would decode to a value that is not bit-identical
/// to the literal `24.1_f32`, making test and display comparisons awkward.
fn volts_from_raw(raw: u16) -> f32 {
    raw as f32 / 10.0
}

/// Assemble the 10-bit voltage from a slot 0 marker and value byte.
fn raw_voltage(data: [u8; 2]) -> u16 {
    (((data[0] & MARKER_VALUE_HIGH_MASK) as u16) << 8) | data[1] as u16
}

impl SlotResponse {
    /// Decode a three-byte slot response, or `None` if byte 0 is not a valid
    /// slot ID.
    pub fn decode(bytes: &[u8; SLOT_LEN]) -> Option<SlotResponse> {
        let index = slot_index(bytes[0])?;
        let data = [bytes[1], bytes[2]];
        let telemetry = match (index, data[0] & MARKER_SENSOR_MASK) {
            (0, MARKER_RX_BATTERY) => Telemetry::RxBattery {
                volts: volts_from_raw(raw_voltage(data)),
                raw: raw_voltage(data),
            },
            (0, MARKER_EXTERNAL_VOLTAGE) => Telemetry::ExternalVoltage {
                volts: volts_from_raw(raw_voltage(data)),
                raw: raw_voltage(data),
            },
            _ => Telemetry::Unknown { data },
        };
        Some(SlotResponse { index, telemetry })
    }

    /// The footer group this slot answers in (`index / 8`).
    pub fn group(&self) -> u8 {
        self.index / SLOTS_PER_GROUP as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The full ID table from `doc/spec.md` §5.1, laid out group by group.
    /// Kept here (and only here) so the generating formula stays pinned to the
    /// documented values.
    const SPEC_TABLE: [u8; SLOT_COUNT] = [
        0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, // group 0
        0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, // group 1
        0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, // group 2
        0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, // group 3
    ];

    #[test]
    fn slot_ids_match_spec_table() {
        for (index, &expected) in SPEC_TABLE.iter().enumerate() {
            assert_eq!(slot_id(index as u8), expected, "slot {index}");
        }
    }

    #[test]
    fn slot_ids_are_unique() {
        for (i, a) in SPEC_TABLE.iter().enumerate() {
            for (j, b) in SPEC_TABLE.iter().enumerate().skip(i + 1) {
                assert_ne!(a, b, "slots {i} and {j} collide");
            }
        }
    }

    #[test]
    fn slot_id_round_trips() {
        for index in 0..SLOT_COUNT as u8 {
            assert_eq!(slot_index(slot_id(index)), Some(index));
        }
    }

    #[test]
    fn frame_start_byte_is_not_a_slot_id() {
        // The parser relies on this to tell a frame head from a slot head.
        assert_eq!(slot_index(crate::frame::START), None);
    }

    #[test]
    fn slot_index_rejects_ids_without_low_bits_set() {
        for id in [0x00u8, 0x01, 0x02, 0xC0, 0xC4, 0xF1] {
            assert_eq!(slot_index(id), None, "0x{id:02X}");
        }
    }

    #[test]
    fn exactly_32_of_256_bytes_are_slot_ids() {
        let valid: [u8; SLOT_COUNT] = core::array::from_fn(|i| slot_id(i as u8));
        for byte in 0u8..=0xFF {
            let expected = valid.contains(&byte);
            assert_eq!(
                slot_index(byte).is_some(),
                expected,
                "0x{byte:02X} misclassified"
            );
            // Every valid ID matches the 0b_____011 pattern, and only those do.
            assert_eq!(byte & 0b111 == 0b011, expected, "0x{byte:02X} pattern");
        }
    }

    #[test]
    fn decodes_captured_rx_battery() {
        // Transmitter showed Rx-Batt 4.9 V for these bytes.
        let r = SlotResponse::decode(&[0x03, 0xC0, 0x31]).unwrap();
        assert_eq!(r.index, 0);
        assert_eq!(r.group(), 0);
        assert_eq!(
            r.telemetry,
            Telemetry::RxBattery {
                volts: 4.9,
                raw: 49
            }
        );
    }

    #[test]
    fn decodes_captured_external_voltage() {
        // Transmitter showed Ext-Volt 24.1 V for these bytes.
        let r = SlotResponse::decode(&[0x03, 0xC4, 0xF1]).unwrap();
        assert_eq!(r.index, 0);
        assert_eq!(
            r.telemetry,
            Telemetry::ExternalVoltage {
                volts: 24.1,
                raw: 241
            }
        );
        assert_eq!(r.telemetry.volts(), Some(24.1));
    }

    /// The capture that settled the encoding: a bench supply set to 26.0 V on
    /// the external input, which is above the 25.5 V an 8-bit value can reach.
    /// The marker moved `0xC4` → `0xC5`, putting bit 8 of the value there.
    #[test]
    fn decodes_external_voltage_above_the_8bit_ceiling() {
        let r = SlotResponse::decode(&[0x03, 0xC5, 0x04]).unwrap();
        assert_eq!(
            r.telemetry,
            Telemetry::ExternalVoltage {
                volts: 26.0,
                raw: 260
            }
        );
        // The same capture jittered by one LSB, as it did at 24.1 V.
        let r = SlotResponse::decode(&[0x03, 0xC5, 0x03]).unwrap();
        assert_eq!(r.telemetry.volts(), Some(25.9));
    }

    /// Rx-Batt uses the same two marker bits, so a high supply would decode
    /// the same way. Not observed — the rail is a regulated 5 V — but the
    /// encoding is shared and must not be special-cased.
    #[test]
    fn rx_battery_uses_the_same_high_bits() {
        let r = SlotResponse::decode(&[0x03, 0xC1, 0x2C]).unwrap();
        assert_eq!(r.telemetry.volts(), Some(30.0));
    }

    #[test]
    fn voltage_scale_is_exact_at_endpoints() {
        assert_eq!(volts_from_raw(0), 0.0);
        assert_eq!(volts_from_raw(1023), VOLT_MAX_V);
    }

    #[test]
    fn marker_high_bits_are_the_value_not_the_sensor() {
        // All four Ext-Volt markers select the same sensor.
        for (marker, expect) in [(0xC4, 0.4), (0xC5, 26.0), (0xC6, 51.6), (0xC7, 77.2)] {
            let r = SlotResponse::decode(&[0x03, marker, 0x04]).unwrap();
            let volts = match r.telemetry {
                Telemetry::ExternalVoltage { volts, .. } => volts,
                other => panic!("0x{marker:02X} decoded as {other:?}"),
            };
            assert_eq!(volts, expect, "marker 0x{marker:02X}");
        }
    }

    #[test]
    fn unexpected_slot0_marker_is_unknown_not_a_guess() {
        let r = SlotResponse::decode(&[0x03, 0xC8, 0x12]).unwrap();
        assert_eq!(r.telemetry, Telemetry::Unknown { data: [0xC8, 0x12] });
        assert_eq!(r.telemetry.volts(), None);
    }

    #[test]
    fn non_zero_slots_are_unknown() {
        // slot 4 (0x23) with what looks like a voltage marker must not be
        // decoded as one: the marker meaning is only verified for slot 0.
        let r = SlotResponse::decode(&[0x23, 0xC4, 0xF1]).unwrap();
        assert_eq!(r.index, 4);
        assert_eq!(r.telemetry, Telemetry::Unknown { data: [0xC4, 0xF1] });
    }

    #[test]
    fn group_derives_from_index() {
        for (index, group) in [
            (0u8, 0u8),
            (7, 0),
            (8, 1),
            (15, 1),
            (16, 2),
            (24, 3),
            (31, 3),
        ] {
            let r = SlotResponse::decode(&[slot_id(index), 0x00, 0x00]).unwrap();
            assert_eq!(r.group(), group, "slot {index}");
        }
    }

    #[test]
    fn decode_rejects_invalid_id() {
        assert!(SlotResponse::decode(&[0x0F, 0xC0, 0x31]).is_none());
    }
}
