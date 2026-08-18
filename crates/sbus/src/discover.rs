//! CH348 port discovery, without depending on fixed device names.
//!
//! `/dev/ttyCH9344USBn` numbering follows enumeration order, not the physical
//! UART wiring, so the SBUS port is not reliably index 6 in the device name.
//! The ch9344 driver exposes a `GETUARTINDEX` ioctl that returns the physical
//! UART number, which is what the board spec assigns functions to.
//!
//! This mirrors `nm_board/ch348/test/ch348hw.py::find_device`, with one
//! deliberate difference: where the Python falls back to enumeration order if
//! the ioctl is unavailable, this returns an error instead. Silently opening
//! the wrong UART — a 4 Mbps RS485 motor bus, say — is worse than failing.

use std::fs;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};

use crate::error::{Error, Result};

/// USB vendor ID of WCH (CH348 / CH9344).
pub const CH348_VID: u16 = 0x1A86;

/// Physical UART index carrying S.BUS on namiashi rev2.
pub const SBUS_UART_INDEX: u16 = 6;

/// `_IOC(_IOC_READ, 'W', 0x85, 2)` — read the physical UART index.
#[cfg(target_os = "linux")]
const IOCTL_GETUARTINDEX: libc::c_ulong = 0x8002_5785;

/// A CH348 tty together with the physical UART it drives.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ch348Port {
    /// Device node, e.g. `/dev/ttyCH9344USB6`.
    pub path: PathBuf,
    /// Physical UART index from `GETUARTINDEX`.
    pub uart_index: u16,
}

/// Read the physical UART index of an open CH348 tty.
#[cfg(target_os = "linux")]
fn uart_index(path: &Path) -> Result<u16> {
    use std::fs::OpenOptions;
    use std::os::unix::fs::OpenOptionsExt;

    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
        .open(path)?;

    let mut index: u16 = 0;
    // SAFETY: the ioctl writes a single u16 through the pointer, matching the
    // 2-byte size encoded in the request code.
    let rc = unsafe {
        libc::ioctl(
            file.as_raw_fd(),
            IOCTL_GETUARTINDEX as _,
            &mut index as *mut u16,
        )
    };
    if rc < 0 {
        return Err(Error::Discovery(format!(
            "{}: GETUARTINDEX ioctl failed: {}",
            path.display(),
            std::io::Error::last_os_error()
        )));
    }
    Ok(index)
}

#[cfg(not(target_os = "linux"))]
fn uart_index(path: &Path) -> Result<u16> {
    Err(Error::Discovery(format!(
        "{}: CH348 UART index discovery is Linux-only; pass an explicit port",
        path.display()
    )))
}

/// Walk up from a tty's sysfs device link to the USB device holding `idVendor`.
fn usb_vendor_of(tty_name: &str) -> Option<u16> {
    let mut dir = fs::canonicalize(format!("/sys/class/tty/{tty_name}/device")).ok()?;
    for _ in 0..8 {
        let vid_path = dir.join("idVendor");
        if vid_path.exists() {
            let text = fs::read_to_string(vid_path).ok()?;
            return u16::from_str_radix(text.trim(), 16).ok();
        }
        if !dir.pop() {
            break;
        }
    }
    None
}

fn candidate_nodes() -> Vec<PathBuf> {
    let mut nodes = Vec::new();
    if let Ok(entries) = fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("ttyCH9344USB") || name.starts_with("ttyUSB") {
                nodes.push(entry.path());
            }
        }
    }
    nodes.sort();
    nodes
}

/// Every CH348 tty on the system, with its physical UART index.
///
/// Nodes whose vendor is not WCH are skipped silently; nodes that are WCH but
/// refuse the ioctl are logged and skipped, since they may be a different
/// driver generation.
pub fn list_ch348_ports() -> Result<Vec<Ch348Port>> {
    let mut ports = Vec::new();
    for path in candidate_nodes() {
        let Some(name) = path.file_name().map(|n| n.to_string_lossy().into_owned()) else {
            continue;
        };
        if usb_vendor_of(&name) != Some(CH348_VID) {
            continue;
        }
        match uart_index(&path) {
            Ok(uart_index) => ports.push(Ch348Port { path, uart_index }),
            Err(e) => log::warn!("skipping {}: {e}", path.display()),
        }
    }
    ports.sort_by_key(|p| p.uart_index);
    Ok(ports)
}

/// The device node for the S.BUS UART (physical index 6).
pub fn find_sbus_port() -> Result<PathBuf> {
    find_uart(SBUS_UART_INDEX)
}

/// The device node for a given physical UART index.
pub fn find_uart(index: u16) -> Result<PathBuf> {
    let ports = list_ch348_ports()?;
    if ports.is_empty() {
        return Err(Error::Discovery(format!(
            "no CH348 (VID {CH348_VID:#06x}) tty found; is the ch9344 driver loaded?"
        )));
    }
    let mut matching = ports.iter().filter(|p| p.uart_index == index);
    let Some(found) = matching.next() else {
        let available: Vec<String> = ports.iter().map(|p| p.uart_index.to_string()).collect();
        return Err(Error::Discovery(format!(
            "no CH348 tty with UART index {index}; found indices [{}]",
            available.join(", ")
        )));
    };
    if let Some(other) = matching.next() {
        return Err(Error::Discovery(format!(
            "UART index {index} is ambiguous: {} and {} both claim it; \
             connect a single board or pass an explicit port",
            found.path.display(),
            other.path.display()
        )));
    }
    Ok(found.path.clone())
}
