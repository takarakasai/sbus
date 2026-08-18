//! Byte-oriented resynchronising parser for the S.BUS / S.BUS2 stream.
//!
//! S.BUS has no checksum and, on S.BUS2, telemetry slot responses arrive
//! about 2 ms after the control frame that selected their group. A decoder
//! therefore cannot validate a frame in isolation, nor assume a frame and its
//! slots land in the same read. [`Parser`] consumes the stream one byte at a
//! time and emits [`Event`]s as units complete.
//!
//! Dropped bytes are reported as [`Event::Desync`] rather than silently
//! discarded: distinguishing genuine garbage from telemetry is the whole
//! problem here, and a caller that cannot see the count has no way to notice
//! it is throwing telemetry away.

use crate::frame::{Footer, Frame, FRAME_LEN, START};
use crate::slot::{slot_index, SlotResponse, SLOT_LEN};

/// Something the parser recognised in the byte stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    /// A complete 25-byte control frame.
    Frame {
        /// Decoded contents.
        frame: Frame,
        /// The bytes as received, for hexdumps.
        raw: [u8; FRAME_LEN],
    },
    /// A telemetry slot response following an S.BUS2 frame.
    Slot {
        /// Footer group of the frame this answers.
        group: u8,
        /// Decoded contents.
        response: SlotResponse,
        /// The bytes as received, for hexdumps.
        raw: [u8; SLOT_LEN],
    },
    /// One byte dropped while resynchronising.
    Desync {
        /// The discarded byte.
        byte: u8,
    },
}

/// Resynchronising S.BUS / S.BUS2 stream parser.
///
/// Allocation-free: the internal buffer never needs to exceed one frame, see
/// [`Parser::push`].
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
    /// Group of the last S.BUS2 frame, while slot responses are still expected.
    slot_group: Option<u8>,
}

impl Default for Parser {
    fn default() -> Self {
        Parser::new()
    }
}

impl Parser {
    /// A parser with no buffered bytes and no slot context.
    pub const fn new() -> Parser {
        Parser {
            buf: [0; FRAME_LEN],
            len: 0,
            slot_group: None,
        }
    }

    /// Drop all buffered bytes and slot context.
    pub fn reset(&mut self) {
        self.len = 0;
        self.slot_group = None;
    }

    /// Bytes currently buffered, waiting for more input to classify.
    pub fn buffered(&self) -> usize {
        self.len
    }

    /// Push one byte and return the event it completed, if any.
    ///
    /// At most one event per byte. The buffer holds at most [`FRAME_LEN`]
    /// bytes because no completed unit is ever left in it:
    ///
    /// - a slot response is consumed as soon as three bytes are present with a
    ///   slot ID at the head, so that case never accumulates further;
    /// - a frame is consumed, or its head byte dropped, the moment the buffer
    ///   reaches [`FRAME_LEN`];
    /// - dropping a head byte also clears the slot context, so the 24 bytes
    ///   left behind cannot immediately complete a slot either.
    pub fn push(&mut self, byte: u8) -> Option<Event> {
        debug_assert!(self.len < FRAME_LEN, "buffer should never be full on entry");
        self.buf[self.len] = byte;
        self.len += 1;

        // 1. Slot response, but only while a preceding S.BUS2 frame makes one
        //    plausible. Slot IDs and START occupy disjoint byte values, so this
        //    cannot swallow a frame head.
        if let Some(group) = self.slot_group {
            if self.len >= SLOT_LEN && slot_index(self.buf[0]).is_some() {
                let mut raw = [0u8; SLOT_LEN];
                raw.copy_from_slice(&self.buf[..SLOT_LEN]);
                self.consume(SLOT_LEN);
                // The ID was already validated, so decode cannot fail.
                let response = SlotResponse::decode(&raw)?;
                return Some(Event::Slot {
                    group,
                    response,
                    raw,
                });
            }
        }

        // 2. Not enough bytes to judge a frame yet.
        if self.len < FRAME_LEN {
            return None;
        }

        // 3. A well-formed frame at the head.
        if self.buf[0] == START {
            if let Some(footer) = Footer::from_byte(self.buf[FRAME_LEN - 1]) {
                let raw = self.buf;
                self.consume(FRAME_LEN);
                self.slot_group = footer.group();
                let frame = Frame::decode(&raw).ok()?;
                return Some(Event::Frame { frame, raw });
            }
        }

        // 4. Resynchronise: drop the head byte. The slot context goes with it,
        //    since a lost frame boundary means the group is no longer known.
        let byte = self.buf[0];
        self.consume(1);
        self.slot_group = None;
        Some(Event::Desync { byte })
    }

    /// Push a slice, invoking `f` for each event in order.
    ///
    /// Convenience over [`Parser::push`] for callers that read in chunks.
    pub fn push_slice(&mut self, bytes: &[u8], mut f: impl FnMut(Event)) {
        for &byte in bytes {
            if let Some(event) = self.push(byte) {
                f(event);
            }
        }
    }

    fn consume(&mut self, count: usize) {
        self.buf.copy_within(count..self.len, 0);
        self.len -= count;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;
    use crate::slot::Telemetry;

    const FRAME_G0: [u8; FRAME_LEN] = [
        0x0F, 0xF9, 0x5B, 0xDF, 0x02, 0xED, 0x07, 0x04, 0x20, 0x00, 0x1F, 0xF8, 0x40, 0x00, 0x3E,
        0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x04,
    ];
    const SLOT_EXT: [u8; SLOT_LEN] = [0x03, 0xC4, 0xF1];
    const SLOT_RX: [u8; SLOT_LEN] = [0x03, 0xC0, 0x31];

    fn frame_with_footer(footer: u8) -> [u8; FRAME_LEN] {
        let mut f = FRAME_G0;
        f[FRAME_LEN - 1] = footer;
        f
    }

    fn run(bytes: &[u8]) -> Vec<Event> {
        let mut parser = Parser::new();
        let mut events = Vec::new();
        parser.push_slice(bytes, |e| events.push(e));
        events
    }

    #[test]
    fn decodes_frame_then_slot() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&SLOT_EXT);

        let events = run(&stream);
        assert_eq!(events.len(), 2);
        match events[0] {
            Event::Frame { frame, raw } => {
                assert_eq!(frame.footer, Footer::Sbus2 { group: 0 });
                assert_eq!(raw, FRAME_G0);
            }
            other => panic!("expected frame, got {other:?}"),
        }
        match events[1] {
            Event::Slot {
                group, response, ..
            } => {
                assert_eq!(group, 0);
                assert_eq!(
                    response.telemetry,
                    Telemetry::ExternalVoltage {
                        volts: 24.1,
                        raw: 0xF1
                    }
                );
            }
            other => panic!("expected slot, got {other:?}"),
        }
    }

    #[test]
    fn slot_bytes_are_not_counted_as_desync() {
        let mut stream = Vec::new();
        for footer in [0x04u8, 0x14, 0x24, 0x34] {
            stream.extend_from_slice(&frame_with_footer(footer));
            if footer == 0x04 {
                stream.extend_from_slice(&SLOT_RX);
            }
        }
        // Trailing frame so the last unit can complete.
        stream.extend_from_slice(&frame_with_footer(0x04));

        let events = run(&stream);
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Desync { .. }))
                .count(),
            0
        );
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Slot { .. }))
                .count(),
            1
        );
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Frame { .. }))
                .count(),
            5
        );
    }

    #[test]
    fn slot_is_ignored_without_a_preceding_sbus2_frame() {
        // A bare slot response with no frame context is garbage, not telemetry.
        let mut stream = Vec::new();
        stream.extend_from_slice(&SLOT_EXT);
        stream.extend_from_slice(&FRAME_G0);

        let events = run(&stream);
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Desync { .. }))
                .count(),
            SLOT_LEN
        );
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Slot { .. }))
                .count(),
            0
        );
    }

    #[test]
    fn sbus1_footer_disables_slot_parsing() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&frame_with_footer(0x00));
        stream.extend_from_slice(&SLOT_EXT);
        stream.extend_from_slice(&FRAME_G0);

        let events = run(&stream);
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Slot { .. }))
                .count(),
            0
        );
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Desync { .. }))
                .count(),
            SLOT_LEN
        );
    }

    #[test]
    fn recovers_after_leading_garbage() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&[0xAA, 0xBB, 0xCC]);
        stream.extend_from_slice(&FRAME_G0);
        stream.extend_from_slice(&SLOT_EXT);

        let events = run(&stream);
        let desync: Vec<u8> = events
            .iter()
            .filter_map(|e| match e {
                Event::Desync { byte } => Some(*byte),
                _ => None,
            })
            .collect();
        assert_eq!(desync, [0xAA, 0xBB, 0xCC]);
        assert!(matches!(events[3], Event::Frame { .. }));
        assert!(matches!(events[4], Event::Slot { .. }));
    }

    #[test]
    fn desync_clears_slot_context() {
        // Frame, then a corrupted byte, then a slot response that must not be
        // trusted because the group is no longer known.
        let mut stream = Vec::new();
        stream.extend_from_slice(&FRAME_G0);
        stream.push(0xAA);
        stream.extend_from_slice(&SLOT_EXT);
        stream.extend_from_slice(&FRAME_G0);

        let events = run(&stream);
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, Event::Slot { .. }))
                .count(),
            0
        );
    }

    #[test]
    fn buffer_never_exceeds_one_frame() {
        let mut stream = Vec::new();
        for footer in [0x04u8, 0x14, 0x24, 0x34, 0x04] {
            stream.extend_from_slice(&frame_with_footer(footer));
            stream.extend_from_slice(&SLOT_RX);
        }
        stream.extend_from_slice(&[0x00, 0x11, 0x22]);

        let mut parser = Parser::new();
        for &byte in &stream {
            parser.push(byte);
            assert!(parser.buffered() < FRAME_LEN, "buffer overran");
        }
    }

    #[test]
    fn one_push_yields_at_most_one_event() {
        // Eight slot responses back to back is the densest legal burst.
        let mut stream = Vec::new();
        stream.extend_from_slice(&FRAME_G0);
        for index in 0..8u8 {
            stream.extend_from_slice(&[crate::slot::slot_id(index), 0x00, 0x00]);
        }

        let mut parser = Parser::new();
        let mut events = 0;
        for &byte in &stream {
            if parser.push(byte).is_some() {
                events += 1;
            }
        }
        // 1 frame + 8 slots, with no event lost to the one-per-push limit.
        assert_eq!(events, 9);
    }

    #[test]
    fn chunk_boundaries_do_not_change_the_event_stream() {
        let mut stream = Vec::new();
        for footer in [0x04u8, 0x14, 0x24, 0x34] {
            stream.extend_from_slice(&frame_with_footer(footer));
            if footer == 0x04 {
                stream.extend_from_slice(&SLOT_EXT);
            }
        }
        let reference = run(&stream);

        // Real reads split wherever the USB packet happened to end, including
        // mid-frame and mid-slot.
        for chunk in [1usize, 2, 3, 7, 13, 24, 25, 26, 64] {
            let mut parser = Parser::new();
            let mut events = Vec::new();
            for part in stream.chunks(chunk) {
                parser.push_slice(part, |e| events.push(e));
            }
            assert_eq!(events, reference, "chunk size {chunk}");
        }
    }

    #[test]
    fn reset_drops_buffered_bytes_and_context() {
        let mut parser = Parser::new();
        for &byte in &FRAME_G0[..10] {
            parser.push(byte);
        }
        assert_eq!(parser.buffered(), 10);
        parser.reset();
        assert_eq!(parser.buffered(), 0);

        // After a reset the stream must resync from scratch.
        let mut events = Vec::new();
        parser.push_slice(&SLOT_EXT, |e| events.push(e));
        assert!(events.is_empty(), "slot accepted without frame context");
    }
}
