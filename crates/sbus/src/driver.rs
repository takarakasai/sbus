//! Synchronous serial driver for receiving S.BUS / S.BUS2.

use std::io::Read;
use std::path::Path;
use std::time::{Duration, Instant};

use sbus_protocol::{Event, Frame, Parser};
use serialport::{DataBits, Parity, SerialPort, StopBits};

use crate::discover;
use crate::error::{Error, Result};
use crate::state::State;

/// S.BUS line rate.
pub const BAUD: u32 = 100_000;

/// Per-`read` poll timeout. Short, so callers can re-check their own deadline.
const READ_POLL_TIMEOUT: Duration = Duration::from_millis(20);

/// Window over which [`State::fps`] is averaged.
const RATE_WINDOW: Duration = Duration::from_secs(1);

/// Receive buffer size. One 15 ms frame period carries at most 49 bytes
/// (frame + eight slot responses), so this holds several periods.
const RX_CHUNK: usize = 256;

/// A receive-only S.BUS / S.BUS2 port.
///
/// The namiashi rev2 SBUS port inverts in hardware (U15) and has no transmit
/// path wired to the connector, so this driver never writes and never inverts
/// in software.
pub struct Sbus {
    port: Box<dyn SerialPort>,
    parser: Parser,
    state: State,
    rx: [u8; RX_CHUNK],
    rate_window_start: Instant,
    rate_count: u32,
}

impl Sbus {
    /// Open an explicit device node at 100 000 baud, 8E2.
    pub fn open(path: impl AsRef<Path>) -> Result<Sbus> {
        let path = path.as_ref();
        let port = serialport::new(path.to_string_lossy(), BAUD)
            .data_bits(DataBits::Eight)
            .parity(Parity::Even)
            .stop_bits(StopBits::Two)
            .timeout(READ_POLL_TIMEOUT)
            .open()?;
        log::debug!("opened {} at {BAUD} baud 8E2", path.display());
        Ok(Sbus {
            port,
            parser: Parser::new(),
            state: State::default(),
            rx: [0; RX_CHUNK],
            rate_window_start: Instant::now(),
            rate_count: 0,
        })
    }

    /// Locate the CH348 S.BUS UART and open it.
    pub fn open_auto() -> Result<Sbus> {
        Sbus::open(discover::find_sbus_port()?)
    }

    /// Latest values and counters.
    pub fn state(&self) -> &State {
        &self.state
    }

    /// Read whatever bytes are available, decode them, and invoke `f` once per
    /// event. Returns the number of events produced.
    ///
    /// Blocks for at most [`READ_POLL_TIMEOUT`]; an idle port yields `Ok(0)`
    /// rather than an error, since a quiet line is normal when the
    /// transmitter is off.
    pub fn poll(&mut self, f: impl FnMut(&Event)) -> Result<usize> {
        self.poll_raw(|_| {}, f)
    }

    /// Like [`Sbus::poll`], but hands the bytes just read to `sink` before any
    /// event is delivered.
    ///
    /// Capture tools want the stream exactly as it arrived, including bytes
    /// the decoder rejects and bytes still sitting in the parser at the end —
    /// neither of which can be reconstructed from the event sequence alone.
    pub fn poll_raw(
        &mut self,
        mut sink: impl FnMut(&[u8]),
        mut f: impl FnMut(&Event),
    ) -> Result<usize> {
        let read = match self.port.read(&mut self.rx) {
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => 0,
            Err(e) => return Err(Error::Io(e)),
        };
        sink(&self.rx[..read]);

        let mut events = 0;
        for i in 0..read {
            let byte = self.rx[i];
            if let Some(event) = self.parser.push(byte) {
                self.state.apply(&event);
                if matches!(event, Event::Frame { .. }) {
                    self.rate_count += 1;
                }
                f(&event);
                events += 1;
            }
        }

        let elapsed = self.rate_window_start.elapsed();
        if elapsed >= RATE_WINDOW {
            self.state.fps = self.rate_count as f32 / elapsed.as_secs_f32();
            self.rate_count = 0;
            self.rate_window_start = Instant::now();
        }

        Ok(events)
    }

    /// Block until a control frame arrives or `timeout` elapses.
    pub fn read_frame(&mut self, timeout: Duration) -> Result<Frame> {
        let deadline = Instant::now() + timeout;
        loop {
            let mut got = None;
            self.poll(|event| {
                if let Event::Frame { frame, .. } = event {
                    got = Some(*frame);
                }
            })?;
            if let Some(frame) = got {
                return Ok(frame);
            }
            if Instant::now() >= deadline {
                return Err(Error::Timeout(timeout));
            }
        }
    }

    /// Discard buffered bytes on both the port and the parser.
    pub fn resync(&mut self) -> Result<()> {
        self.port.clear(serialport::ClearBuffer::Input)?;
        self.parser.reset();
        Ok(())
    }
}
