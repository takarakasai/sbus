//! Regression tests against real captures from the bench.
//!
//! These pin the decoder to bytes that were verified on hardware against a
//! transmitter's telemetry display, so the crate stays testable with the
//! transmitter powered off. Expected values come from `doc/spec.md` §6.
//!
//! Two captures, taken minutes apart:
//!
//! - `sbus2_linked_3s.bin` — transmitter on, link healthy.
//! - `sbus2_failsafe_8s.bin` — transmitter off. Every frame asserts failsafe
//!   and frame_lost, channels hold their last values, and telemetry keeps
//!   flowing because the receiver measures its own voltages.

use sbus_protocol::{Event, Footer, Frame, Parser, Telemetry};

const LINKED: &[u8] = include_bytes!("fixtures/sbus2_linked_3s.bin");
const FAILSAFE: &[u8] = include_bytes!("fixtures/sbus2_failsafe_8s.bin");

#[derive(Default, Debug)]
struct Tally {
    frames: usize,
    slots: usize,
    unknown_slots: usize,
    desync_bytes: usize,
    /// Observed range of each voltage. The low bit jitters by 1 LSB, so the
    /// last sample alone is not a stable expectation.
    rx_battery_v: Option<(f32, f32)>,
    external_v: Option<(f32, f32)>,
    groups: [usize; 4],
    failsafe: usize,
    frame_lost: usize,
    first_frame: Option<Frame>,
    last_frame: Option<Frame>,
}

/// Widen a running (min, max) with one sample.
fn observe(range: &mut Option<(f32, f32)>, volts: f32) {
    match range {
        Some((lo, hi)) => {
            *lo = lo.min(volts);
            *hi = hi.max(volts);
        }
        None => *range = Some((volts, volts)),
    }
}

fn decode(bytes: &[u8]) -> Tally {
    let mut parser = Parser::new();
    let mut t = Tally::default();
    parser.push_slice(bytes, |event| match event {
        Event::Frame { frame, .. } => {
            t.frames += 1;
            if let Footer::Sbus2 { group } = frame.footer {
                t.groups[group as usize] += 1;
            }
            t.failsafe += frame.failsafe as usize;
            t.frame_lost += frame.frame_lost as usize;
            t.first_frame.get_or_insert(frame);
            t.last_frame = Some(frame);
        }
        Event::Slot { response, .. } => {
            t.slots += 1;
            match response.telemetry {
                Telemetry::RxBattery { volts, .. } => observe(&mut t.rx_battery_v, volts),
                Telemetry::ExternalVoltage { volts, .. } => observe(&mut t.external_v, volts),
                Telemetry::Unknown { .. } => t.unknown_slots += 1,
            }
        }
        Event::Desync { .. } => t.desync_bytes += 1,
    });
    t
}

#[test]
fn linked_capture() {
    let t = decode(LINKED);
    assert_eq!(t.frames, 199);
    assert_eq!(t.slots, 50);
    assert_eq!(t.unknown_slots, 0);
    // Rx-Batt is a regulated 5 V rail and does not move; Ext-Volt jitters by
    // one LSB. The transmitter displayed 4.9 V and 24.1 V.
    assert_eq!(t.rx_battery_v, Some((4.9, 4.9)));
    assert_eq!(t.external_v, Some((24.0, 24.1)));
    assert_eq!(t.failsafe, 0);
    assert_eq!(t.frame_lost, 0);
    assert_eq!(
        t.first_frame.unwrap().channels,
        [
            1017, 1003, 1035, 1014, 64, 64, 1984, 1984, 64, 1984, 1024, 1024, 1024, 1024, 1024,
            1024
        ]
    );
}

#[test]
fn failsafe_capture() {
    let t = decode(FAILSAFE);
    assert_eq!(t.frames, 532);
    assert_eq!(t.slots, 133);
    assert_eq!(t.unknown_slots, 0);
    // Telemetry survives loss of the RF link: these are the receiver's own
    // measurements, not values relayed from the transmitter. Ext-Volt reads a
    // steady 24.0 V here, one LSB below the linked capture taken minutes
    // earlier.
    assert_eq!(t.rx_battery_v, Some((4.9, 4.9)));
    assert_eq!(t.external_v, Some((24.0, 24.0)));
}

/// With the transmitter off, every frame flags both faults.
#[test]
fn failsafe_capture_flags_every_frame() {
    let t = decode(FAILSAFE);
    assert_eq!(t.failsafe, t.frames);
    assert_eq!(t.frame_lost, t.frames);
}

/// Failsafe holds the last received channel values rather than zeroing them.
#[test]
fn failsafe_capture_holds_channels() {
    let t = decode(FAILSAFE);
    let first = t.first_frame.unwrap();
    let last = t.last_frame.unwrap();
    assert_eq!(first.channels, last.channels);
    assert!(first.channels.iter().all(|&c| c != 0));
}

/// The point of the whole exercise: telemetry must not be mistaken for noise.
///
/// The Python monitor originally reported these same bytes as `skip=249` over
/// five seconds. Anything above a handful of bytes here means slot responses
/// are being dropped again.
#[test]
fn captures_decode_without_losing_bytes() {
    for (name, bytes) in [("linked", LINKED), ("failsafe", FAILSAFE)] {
        let t = decode(bytes);
        assert_eq!(t.desync_bytes, 0, "{name}");
        // A capture starts and stops mid-stream, so a partial unit may be left
        // over at the tail; anything larger means units were missed.
        let consumed = t.frames * 25 + t.slots * 3;
        assert!(
            bytes.len() - consumed < 25,
            "{name}: {} trailing bytes unaccounted for",
            bytes.len() - consumed
        );
    }
}

/// Footer cycles 0 -> 1 -> 2 -> 3, and only group 0 carries a slot response.
#[test]
fn footer_groups_cycle_evenly() {
    for (name, bytes) in [("linked", LINKED), ("failsafe", FAILSAFE)] {
        let t = decode(bytes);
        let g0 = t.groups[0];
        for (i, &count) in t.groups.iter().enumerate() {
            assert!(
                g0.abs_diff(count) <= 1,
                "{name}: group {i} appeared {count} times vs group 0's {g0}"
            );
        }
        assert_eq!(t.groups.iter().sum::<usize>(), t.frames, "{name}");
        assert!(
            t.slots.abs_diff(g0) <= 1,
            "{name}: expected ~one slot response per group-0 frame, got {} for {g0}",
            t.slots
        );
    }
}

/// Both voltage markers appear, in equal numbers.
#[test]
fn voltage_markers_alternate_one_to_one() {
    for (name, bytes) in [("linked", LINKED), ("failsafe", FAILSAFE)] {
        let mut parser = Parser::new();
        let (mut rx, mut ext) = (0usize, 0usize);
        parser.push_slice(bytes, |event| {
            if let Event::Slot { response, .. } = event {
                match response.telemetry {
                    Telemetry::RxBattery { .. } => rx += 1,
                    Telemetry::ExternalVoltage { .. } => ext += 1,
                    Telemetry::Unknown { .. } => {}
                }
            }
        });
        assert!(rx > 0 && ext > 0, "{name}: rx={rx} ext={ext}");
        assert!(rx.abs_diff(ext) <= 1, "{name}: not 1:1: rx={rx} ext={ext}");
    }
}

/// Real reads split the stream at arbitrary offsets; the result must not change.
#[test]
fn replay_is_independent_of_chunking() {
    for (name, bytes) in [("linked", LINKED), ("failsafe", FAILSAFE)] {
        let reference = decode(bytes);
        for chunk in [1usize, 7, 25, 64, 512, 4096] {
            let mut parser = Parser::new();
            let (mut frames, mut slots) = (0usize, 0usize);
            for part in bytes.chunks(chunk) {
                parser.push_slice(part, |event| match event {
                    Event::Frame { .. } => frames += 1,
                    Event::Slot { .. } => slots += 1,
                    Event::Desync { .. } => {}
                });
            }
            assert_eq!(frames, reference.frames, "{name} chunk {chunk}");
            assert_eq!(slots, reference.slots, "{name} chunk {chunk}");
        }
    }
}
