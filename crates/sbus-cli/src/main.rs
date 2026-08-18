//! `sbus-monitor` — live monitor, raw capture and offline replay for
//! Futaba S.BUS / S.BUS2.
//!
//! The Rust counterpart of `nm_board/ch348/test/sbus_monitor.py`, plus a
//! `replay` subcommand so captured streams can be decoded without hardware.

mod render;

use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::{Parser as ClapParser, Subcommand};
use sbus::{Event, Sbus, State};
use sbus_protocol::{Parser, Telemetry};

#[derive(ClapParser, Debug)]
#[command(name = "sbus-monitor", about, version)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Decode the live stream and display it.
    Monitor(MonitorArgs),
    /// Save raw bytes from the port to a file, for later replay.
    Dump(DumpArgs),
    /// Decode a previously dumped file. Needs no hardware.
    Replay(ReplayArgs),
    /// List CH348 ttys with their physical UART index.
    Ports,
}

#[derive(clap::Args, Debug)]
struct MonitorArgs {
    /// Device node. Defaults to the auto-discovered CH348 SBUS UART.
    #[arg(long)]
    port: Option<PathBuf>,
    /// Also show the microsecond conversion.
    #[arg(long)]
    us: bool,
    /// Stop after this many seconds.
    #[arg(long)]
    seconds: Option<f64>,
    /// Stop after this many frames.
    #[arg(long)]
    count: Option<u64>,
    /// One line per frame instead of a redrawn display.
    #[arg(long)]
    plain: bool,
    /// Hexdump every frame and slot response.
    #[arg(long)]
    raw: bool,
}

#[derive(clap::Args, Debug)]
struct DumpArgs {
    /// Device node. Defaults to the auto-discovered CH348 SBUS UART.
    #[arg(long)]
    port: Option<PathBuf>,
    /// Capture duration in seconds.
    #[arg(long, default_value_t = 5.0)]
    seconds: f64,
    /// Output file.
    #[arg(long, short)]
    out: PathBuf,
}

#[derive(clap::Args, Debug)]
struct ReplayArgs {
    /// File of raw bytes, as written by `dump`.
    file: PathBuf,
    /// Hexdump every frame and slot response.
    #[arg(long)]
    raw: bool,
    /// One line per frame.
    #[arg(long)]
    plain: bool,
}

fn main() {
    env_logger::init();
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Monitor(args) => monitor(args),
        Command::Dump(args) => dump(args),
        Command::Replay(args) => replay(args),
        Command::Ports => ports(),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn open(port: Option<PathBuf>) -> sbus::Result<Sbus> {
    match port {
        Some(path) => Sbus::open(path),
        None => Sbus::open_auto(),
    }
}

fn ports() -> sbus::Result<()> {
    let found = sbus::list_ch348_ports()?;
    if found.is_empty() {
        println!("no CH348 tty found");
        return Ok(());
    }
    for p in found {
        let note = if p.uart_index == sbus::SBUS_UART_INDEX {
            "  <- SBUS"
        } else {
            ""
        };
        println!("UART{:<2} {}{note}", p.uart_index, p.path.display());
    }
    Ok(())
}

/// Print the hexdump line for one event, matching the Python monitor's layout.
fn dump_event(event: &Event) {
    match event {
        Event::Frame { raw, .. } => println!("{}", render::hex(raw)),
        Event::Slot {
            group,
            response,
            raw,
        } => {
            let tag = match response.telemetry {
                Telemetry::RxBattery { volts, .. } => format!("rx_batt={volts:.1}V"),
                Telemetry::ExternalVoltage { volts, .. } => format!("ext_volt={volts:.1}V"),
                Telemetry::Unknown { data } => {
                    format!("slot{} unknown data {}", response.index, render::hex(&data))
                }
            };
            println!("  slot g{group}: {}  {tag}", render::hex(raw));
        }
        Event::Desync { byte } => println!("  desync: {byte:02X}"),
    }
}

fn monitor(args: MonitorArgs) -> sbus::Result<()> {
    let mut sbus = open(args.port)?;
    let start = Instant::now();
    let deadline = args.seconds.map(Duration::from_secs_f64);
    let redraw = !args.plain && !args.raw;

    let mut stdout = std::io::stdout();
    if redraw {
        let _ = write!(stdout, "{CSI}?25l", CSI = render::CSI);
    }

    let mut previous_lines = 0usize;
    let mut last_draw = Instant::now();
    let mut done = false;

    while !done {
        let mut frames = Vec::new();
        sbus.poll(|event| {
            if args.raw {
                dump_event(event);
            }
            if let Event::Frame { frame, .. } = event {
                frames.push(*frame);
            }
        })?;

        for frame in &frames {
            if args.plain && !args.raw {
                println!("{}", render::plain_frame(frame, sbus.state()));
            }
        }
        if let Some(limit) = args.count {
            if sbus.state().counters.frames >= limit {
                done = true;
            }
        }

        if redraw && last_draw.elapsed() >= Duration::from_millis(50) {
            let lines = render::lines(sbus.state(), args.us);
            if previous_lines > 0 {
                let _ = write!(stdout, "{}{}A", render::CSI, previous_lines);
            }
            for line in &lines {
                let _ = writeln!(stdout, "{}2K{line}", render::CSI);
            }
            previous_lines = lines.len();
            let _ = stdout.flush();
            last_draw = Instant::now();
        }

        if let Some(limit) = deadline {
            if start.elapsed() >= limit {
                done = true;
            }
        }
    }

    if redraw {
        let _ = write!(stdout, "{CSI}?25h", CSI = render::CSI);
        let _ = stdout.flush();
    }
    println!("\n{}", render::summary(sbus.state()));
    Ok(())
}

fn dump(args: DumpArgs) -> sbus::Result<()> {
    let mut sbus = open(args.port)?;
    let mut captured = Vec::new();
    let start = Instant::now();
    let limit = Duration::from_secs_f64(args.seconds);

    // Capture the bytes as read, not the decoded events: a dump is meant to be
    // a faithful record, including anything the decoder would have rejected.
    while start.elapsed() < limit {
        sbus.poll_raw(|bytes| captured.extend_from_slice(bytes), |_| {})?;
    }

    fs::write(&args.out, &captured)?;
    println!("wrote {} bytes to {}", captured.len(), args.out.display());
    println!("{}", render::summary(sbus.state()));
    Ok(())
}

fn replay(args: ReplayArgs) -> sbus::Result<()> {
    let bytes = fs::read(&args.file)?;
    let mut parser = Parser::new();
    let mut state = State::default();

    parser.push_slice(&bytes, |event| {
        state.apply(&event);
        if args.raw {
            dump_event(&event);
        } else if args.plain {
            if let Event::Frame { frame, .. } = &event {
                println!("{}", render::plain_frame(frame, &state));
            }
        }
    });

    println!(
        "{} ({} bytes)\n{}",
        args.file.display(),
        bytes.len(),
        render::summary(&state)
    );
    Ok(())
}
