//! Receive-only driver for Futaba S.BUS / S.BUS2 over an inverted TTL serial
//! port, targeting the namiashi rev2 (CH348L) SBUS input.
//!
//! Wire decoding lives in [`sbus_protocol`]; this crate adds the serial port,
//! CH348 port discovery, and a small aggregated [`State`].
//!
//! # Example
//!
//! ```no_run
//! use std::time::Duration;
//! use sbus::Sbus;
//!
//! let mut sbus = Sbus::open_auto()?;
//! let frame = sbus.read_frame(Duration::from_secs(1))?;
//! println!("CH1 = {}", frame.channels[0]);
//! println!("Ext-Volt = {:?} V", sbus.state().external_v);
//! # Ok::<(), sbus::Error>(())
//! ```
//!
//! The transmit direction is intentionally absent: CN2 carries no TX line, so
//! this port cannot poll S.BUS2 telemetry sensors — it only observes what the
//! receiver already puts on the wire.

pub mod discover;
pub mod driver;
pub mod error;
pub mod state;

pub use discover::{find_sbus_port, list_ch348_ports, Ch348Port, CH348_VID, SBUS_UART_INDEX};
pub use driver::{Sbus, BAUD};
pub use error::{Error, Result};
pub use state::{Counters, State};

pub use sbus_protocol as protocol;
pub use sbus_protocol::{Event, Footer, Frame, Parser, SlotResponse, Telemetry};
