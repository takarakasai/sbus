//! Protocol layer for receiving Futaba S.BUS / S.BUS2 on an inverted TTL UART.
//!
//! This crate is `no_std`, allocation-free and performs no I/O. It decodes a
//! byte stream into control frames and S.BUS2 telemetry slot responses; wire
//! I/O is the caller's responsibility (see the `sbus` crate for a
//! `serialport`-backed driver).
//!
//! See `doc/spec.md` in the repository for the measured wire specification
//! this implementation is derived from.
//!
//! # Wire format
//!
//! ## Control frame — 25 bytes, no checksum
//!
//! ```text
//! offset  0     1                      22   23     24
//!        +-----+----------------------+----+------+--------+
//!        | 0x0F| 16ch x 11bit = 22B        | flags| footer |
//!        +-----+----------------------+----+------+--------+
//! ```
//!
//! The footer distinguishes plain S.BUS (`0x00`) from S.BUS2
//! (`0x04`/`0x14`/`0x24`/`0x34`, encoding telemetry slot group 0..=3).
//!
//! ## S.BUS2 telemetry slot response — 3 bytes
//!
//! ```text
//! +---------+-------+-------+
//! | slot ID | data0 | data1 |
//! +---------+-------+-------+
//! ```
//!
//! Slot responses arrive roughly 2 ms *after* the control frame whose footer
//! selected their group, so a decoder cannot assume frame and slots land in
//! the same read. [`Parser`] handles the interleaving.
//!
//! # Example
//!
//! ```
//! use sbus_protocol::{Event, Parser, Telemetry};
//!
//! let mut parser = Parser::new();
//! let mut ext_volts = None;
//!
//! // A group-0 frame followed by its slot0 Ext-Volt response.
//! let stream = [
//!     0x0F, 0xF9, 0x5B, 0xDF, 0x02, 0xED, 0x07, 0x04, 0x20, 0x00, 0x1F, 0xF8,
//!     0x40, 0x00, 0x3E, 0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00,
//!     0x04, // footer: S.BUS2 group 0
//!     0x03, 0xC4, 0xF1, // slot0: Ext-Volt 24.1 V
//! ];
//!
//! for byte in stream {
//!     if let Some(Event::Slot { response, .. }) = parser.push(byte) {
//!         if let Telemetry::ExternalVoltage { volts, .. } = response.telemetry {
//!             ext_volts = Some(volts);
//!         }
//!     }
//! }
//! assert_eq!(ext_volts, Some(24.1));
//! ```

#![no_std]

pub mod frame;
pub mod parser;
pub mod slot;

pub use frame::{raw_to_us, Footer, Frame, FrameError, FRAME_LEN, START};
pub use parser::{Event, Parser};
pub use slot::{
    slot_id, slot_index, SlotResponse, Telemetry, MARKER_EXTERNAL_VOLTAGE, MARKER_RX_BATTERY,
    SLOT_COUNT, SLOT_LEN, VOLT_LSB_V,
};
