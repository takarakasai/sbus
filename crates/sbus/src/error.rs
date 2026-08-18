//! Error types for the S.BUS driver.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial port: {0}")]
    SerialPort(#[from] serialport::Error),

    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    #[error("port discovery: {0}")]
    Discovery(String),

    #[error("timed out after {0:?} waiting for a control frame")]
    Timeout(std::time::Duration),
}

pub type Result<T> = std::result::Result<T, Error>;
