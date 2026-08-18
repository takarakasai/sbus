//! Terminal rendering for the live monitor.

use sbus::State;
use sbus_protocol::{raw_to_us, Frame};

/// ANSI control sequence introducer.
pub const CSI: &str = "\x1b[";

const BAR_WIDTH_PLAIN: usize = 14;
const BAR_WIDTH_US: usize = 10;

fn bar(raw: u16, width: usize) -> String {
    let filled = (raw as usize * width) / 2047;
    let mut s = String::with_capacity(width * 3);
    for i in 0..width {
        s.push(if i < filled { '█' } else { '·' });
    }
    s
}

fn volts(value: Option<f32>) -> String {
    match value {
        Some(v) => format!("{v:5.1}V"),
        None => "  ---".to_string(),
    }
}

fn flag(value: bool) -> &'static str {
    if value {
        "●"
    } else {
        "○"
    }
}

fn warn(value: bool) -> &'static str {
    if value {
        "⚠YES"
    } else {
        "no"
    }
}

/// The full monitor display as individual lines.
pub fn lines(state: &State, show_us: bool) -> Vec<String> {
    let c = &state.counters;
    let mut out = vec![
        format!(
            "SBUS monitor  100000 8E2   {:5.1} fps   frames={} slots={} desync={}",
            state.fps, c.frames, c.slots, c.desync_bytes
        ),
        match state.frame {
            Some(f) => format!(
                "CH17:{}  CH18:{}   FRAME_LOST:{}   FAILSAFE:{}",
                flag(f.ch17),
                flag(f.ch18),
                warn(f.frame_lost),
                warn(f.failsafe)
            ),
            None => "CH17:-  CH18:-   FRAME_LOST:-   FAILSAFE:-".to_string(),
        },
        format!(
            "{}  Rx-Batt:{}  Ext-Volt:{}{}",
            if state.sbus2 { "S.BUS2" } else { "S.BUS1" },
            volts(state.rx_battery_v),
            volts(state.external_v),
            if c.unknown_slots > 0 {
                format!("   unknown slots={}", c.unknown_slots)
            } else {
                String::new()
            }
        ),
        "-".repeat(62),
    ];

    match &state.frame {
        None => {
            out.push("  (SBUS フレーム待ち... 送信機/受信機を確認してください)".to_string());
            out.push(String::new());
        }
        Some(frame) => {
            for row in 0..8 {
                let mut cells = Vec::with_capacity(2);
                for k in [row * 2, row * 2 + 1] {
                    let v = frame.channels[k];
                    cells.push(if show_us {
                        format!(
                            "CH{:>2} {v:4} {:4}us {}",
                            k + 1,
                            raw_to_us(v),
                            bar(v, BAR_WIDTH_US)
                        )
                    } else {
                        format!("CH{:>2} {v:4} {}", k + 1, bar(v, BAR_WIDTH_PLAIN))
                    });
                }
                out.push(format!("  {}", cells.join("   ")));
            }
        }
    }
    out
}

/// One line per frame, for `--plain`.
pub fn plain_frame(frame: &Frame, state: &State) -> String {
    let channels: Vec<String> = frame.channels.iter().map(|v| format!("{v:4}")).collect();
    format!(
        "CH {}  d17={} d18={} lost={} fs={}  rx={} ext={}",
        channels.join(" "),
        frame.ch17 as u8,
        frame.ch18 as u8,
        frame.frame_lost as u8,
        frame.failsafe as u8,
        volts(state.rx_battery_v).trim(),
        volts(state.external_v).trim()
    )
}

/// Space-separated uppercase hex, for `--raw`.
pub fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Closing summary, shared by every subcommand.
pub fn summary(state: &State) -> String {
    let c = &state.counters;
    format!(
        "frames={} slots={} unknown={} desync={} fps={:.1}\n\
         Rx-Batt={} Ext-Volt={}",
        c.frames,
        c.slots,
        c.unknown_slots,
        c.desync_bytes,
        state.fps,
        volts(state.rx_battery_v).trim(),
        volts(state.external_v).trim()
    )
}
