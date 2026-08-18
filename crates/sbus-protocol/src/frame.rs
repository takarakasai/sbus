//! Control-frame decoding (25 bytes, 16 analog channels + flags + footer).

/// Total length of a control frame.
pub const FRAME_LEN: usize = 25;

/// First byte of every control frame.
pub const START: u8 = 0x0F;

/// Number of 11-bit analog channels carried in a frame.
pub const CHANNELS: usize = 16;

/// Lowest raw channel value in the nominal 1000-2000 us range.
pub const RAW_MIN: u16 = 172;
/// Highest raw channel value in the nominal 1000-2000 us range.
pub const RAW_MAX: u16 = 1811;

/// Footer byte (offset 24) classification.
///
/// The footer is the only field besides the start byte that constrains frame
/// validity — S.BUS carries no checksum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Footer {
    /// `0x00` — plain S.BUS. No telemetry slots follow this frame.
    Sbus1,
    /// `0x04` / `0x14` / `0x24` / `0x34` — S.BUS2. `group` is 0..=3 and selects
    /// which eight telemetry slots may respond after this frame.
    Sbus2 { group: u8 },
}

impl Footer {
    /// Classify a footer byte, or `None` if it is not a known footer value.
    pub const fn from_byte(byte: u8) -> Option<Footer> {
        match byte {
            0x00 => Some(Footer::Sbus1),
            0x04 => Some(Footer::Sbus2 { group: 0 }),
            0x14 => Some(Footer::Sbus2 { group: 1 }),
            0x24 => Some(Footer::Sbus2 { group: 2 }),
            0x34 => Some(Footer::Sbus2 { group: 3 }),
            _ => None,
        }
    }

    /// The wire byte for this footer.
    pub const fn to_byte(self) -> u8 {
        match self {
            Footer::Sbus1 => 0x00,
            Footer::Sbus2 { group } => (group << 4) | 0x04,
        }
    }

    /// Telemetry slot group, or `None` for plain S.BUS.
    pub const fn group(self) -> Option<u8> {
        match self {
            Footer::Sbus1 => None,
            Footer::Sbus2 { group } => Some(group),
        }
    }
}

/// Reason a 25-byte candidate is not a valid control frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Byte 0 is not [`START`].
    BadStart { found: u8 },
    /// Byte 24 is not a recognised footer value.
    BadFooter { found: u8 },
}

/// A decoded control frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// 16 analog channels, each 0..=2047.
    pub channels: [u16; CHANNELS],
    /// Digital channel 17 (flag bit 0).
    pub ch17: bool,
    /// Digital channel 18 (flag bit 1).
    pub ch18: bool,
    /// Receiver is dropping RF frames (flag bit 2).
    pub frame_lost: bool,
    /// Receiver is in failsafe (flag bit 3).
    pub failsafe: bool,
    /// Footer classification, which also tells whether slots may follow.
    pub footer: Footer,
}

/// Unpack the 22 payload bytes into 16 little-endian 11-bit channels.
///
/// Channels 0..=7 live entirely in `d[0..11]` and channels 8..=15 in
/// `d[11..22]`; the two halves do not share bytes.
fn unpack_channels(d: &[u8; 22]) -> [u16; CHANNELS] {
    // Widen once so the shifts below cannot overflow.
    let b = |i: usize| d[i] as u16;
    let mut ch = [0u16; CHANNELS];
    ch[0] = (b(0) | b(1) << 8) & 0x7FF;
    ch[1] = (b(1) >> 3 | b(2) << 5) & 0x7FF;
    ch[2] = (b(2) >> 6 | b(3) << 2 | b(4) << 10) & 0x7FF;
    ch[3] = (b(4) >> 1 | b(5) << 7) & 0x7FF;
    ch[4] = (b(5) >> 4 | b(6) << 4) & 0x7FF;
    ch[5] = (b(6) >> 7 | b(7) << 1 | b(8) << 9) & 0x7FF;
    ch[6] = (b(8) >> 2 | b(9) << 6) & 0x7FF;
    ch[7] = (b(9) >> 5 | b(10) << 3) & 0x7FF;
    ch[8] = (b(11) | b(12) << 8) & 0x7FF;
    ch[9] = (b(12) >> 3 | b(13) << 5) & 0x7FF;
    ch[10] = (b(13) >> 6 | b(14) << 2 | b(15) << 10) & 0x7FF;
    ch[11] = (b(15) >> 1 | b(16) << 7) & 0x7FF;
    ch[12] = (b(16) >> 4 | b(17) << 4) & 0x7FF;
    ch[13] = (b(17) >> 7 | b(18) << 1 | b(19) << 9) & 0x7FF;
    ch[14] = (b(19) >> 2 | b(20) << 6) & 0x7FF;
    ch[15] = (b(20) >> 5 | b(21) << 3) & 0x7FF;
    ch
}

impl Frame {
    /// Decode a 25-byte candidate.
    ///
    /// Only the start byte and the footer are validated — there is no checksum
    /// in S.BUS, so a frame that passes these checks is not guaranteed intact.
    /// Stream-level confidence comes from staying in sync (see [`crate::Parser`]).
    pub fn decode(bytes: &[u8; FRAME_LEN]) -> Result<Frame, FrameError> {
        if bytes[0] != START {
            return Err(FrameError::BadStart { found: bytes[0] });
        }
        let footer = match Footer::from_byte(bytes[24]) {
            Some(f) => f,
            None => return Err(FrameError::BadFooter { found: bytes[24] }),
        };

        let mut payload = [0u8; 22];
        payload.copy_from_slice(&bytes[1..23]);
        let flags = bytes[23];

        Ok(Frame {
            channels: unpack_channels(&payload),
            ch17: flags & 0x01 != 0,
            ch18: flags & 0x02 != 0,
            frame_lost: flags & 0x04 != 0,
            failsafe: flags & 0x08 != 0,
            footer,
        })
    }

    /// Channel `index` converted to approximate microseconds, or `None` if
    /// `index` is out of range.
    pub fn channel_us(&self, index: usize) -> Option<u16> {
        self.channels.get(index).copied().map(raw_to_us)
    }
}

/// Convert a raw channel value to approximate pulse width in microseconds.
///
/// Maps [`RAW_MIN`]..=[`RAW_MAX`] onto 1000..=2000 us. This is the conventional
/// approximation and depends on transmitter/receiver endpoint settings, so it
/// is for display only — use raw values for control.
pub fn raw_to_us(raw: u16) -> u16 {
    let span = (RAW_MAX - RAW_MIN) as i32; // 1639
    let offset = raw as i32 - RAW_MIN as i32;
    // Round half away from zero without floating point.
    let scaled = if offset >= 0 {
        (offset * 1000 + span / 2) / span
    } else {
        (offset * 1000 - span / 2) / span
    };
    (scaled + 1000).clamp(0, u16::MAX as i32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One real group-0 frame from `tests/fixtures/sbus2_linked_3s.bin`.
    const CAPTURED: [u8; FRAME_LEN] = [
        0x0F, 0xF9, 0x5B, 0xDF, 0x02, 0xED, 0x07, 0x04, 0x20, 0x00, 0x1F, 0xF8, 0x40, 0x00, 0x3E,
        0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x04,
    ];

    #[test]
    fn decode_captured_frame() {
        let f = Frame::decode(&CAPTURED).unwrap();
        // Cross-checked against the Python reference decoder on the same bytes.
        assert_eq!(
            f.channels,
            [
                1017, 1003, 1035, 1014, 64, 64, 1984, 1984, 64, 1984, 1024, 1024, 1024, 1024, 1024,
                1024
            ]
        );
        assert!(!f.ch17 && !f.ch18 && !f.frame_lost && !f.failsafe);
        assert_eq!(f.footer, Footer::Sbus2 { group: 0 });
    }

    #[test]
    fn channels_stay_in_11_bit_range() {
        let f = Frame::decode(&CAPTURED).unwrap();
        assert!(f.channels.iter().all(|&c| c <= 0x7FF));
    }

    #[test]
    fn all_ones_payload_saturates_every_channel() {
        let mut bytes = [0xFFu8; FRAME_LEN];
        bytes[0] = START;
        bytes[23] = 0x00; // flags
        bytes[24] = 0x00; // footer
        let f = Frame::decode(&bytes).unwrap();
        assert_eq!(f.channels, [0x7FF; CHANNELS]);
    }

    #[test]
    fn flag_bits_map_to_fields() {
        let mut bytes = [0u8; FRAME_LEN];
        bytes[0] = START;
        for (bit, expect) in [
            (0x01u8, (true, false, false, false)),
            (0x02, (false, true, false, false)),
            (0x04, (false, false, true, false)),
            (0x08, (false, false, false, true)),
        ] {
            bytes[23] = bit;
            let f = Frame::decode(&bytes).unwrap();
            assert_eq!((f.ch17, f.ch18, f.frame_lost, f.failsafe), expect);
        }
        // High nibble is unused and must not disturb the decoded flags.
        bytes[23] = 0xF0;
        let f = Frame::decode(&bytes).unwrap();
        assert_eq!(
            (f.ch17, f.ch18, f.frame_lost, f.failsafe),
            (false, false, false, false)
        );
    }

    #[test]
    fn footer_round_trip() {
        for byte in [0x00u8, 0x04, 0x14, 0x24, 0x34] {
            let f = Footer::from_byte(byte).unwrap();
            assert_eq!(f.to_byte(), byte);
        }
        assert_eq!(Footer::from_byte(0x00).unwrap().group(), None);
        for (byte, group) in [(0x04u8, 0u8), (0x14, 1), (0x24, 2), (0x34, 3)] {
            assert_eq!(Footer::from_byte(byte).unwrap().group(), Some(group));
        }
    }

    #[test]
    fn footer_rejects_unknown_bytes() {
        // 0x44 would be "group 4", which does not exist (group is 2 bits).
        for byte in [0x01u8, 0x0F, 0x44, 0xFF] {
            assert_eq!(Footer::from_byte(byte), None);
        }
    }

    #[test]
    fn decode_rejects_bad_start_and_footer() {
        let mut bytes = CAPTURED;
        bytes[0] = 0x0E;
        assert_eq!(
            Frame::decode(&bytes),
            Err(FrameError::BadStart { found: 0x0E })
        );

        let mut bytes = CAPTURED;
        bytes[24] = 0x44;
        assert_eq!(
            Frame::decode(&bytes),
            Err(FrameError::BadFooter { found: 0x44 })
        );
    }

    #[test]
    fn raw_to_us_endpoints_and_centre() {
        assert_eq!(raw_to_us(RAW_MIN), 1000);
        assert_eq!(raw_to_us(RAW_MAX), 2000);
        assert_eq!(raw_to_us(992), 1500); // (992-172)*1000/1639 = 500.3
                                          // Values outside the nominal band extrapolate rather than clamp.
        assert_eq!(raw_to_us(0), 895);
        assert_eq!(raw_to_us(2047), 2144);
    }

    #[test]
    fn channel_us_matches_raw_to_us() {
        let f = Frame::decode(&CAPTURED).unwrap();
        assert_eq!(f.channel_us(0), Some(raw_to_us(1017)));
        assert_eq!(f.channel_us(16), None);
    }
}
