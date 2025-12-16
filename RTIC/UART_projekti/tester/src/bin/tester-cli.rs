//! Tester CLI tool.
//!
//! CLI application that allows the developer to send single commands
//! or to run predefined test suites that contains multiple commands.
//!
//! Run:
//!
//! `cargo run --target x86_64-unknown-linux-gnu -- --port /dev/ttyUSB0 counter`
//! `cargo run --target x86_64-unknown-linux-gnu -- --port /dev/ttyUSB0 set-time 12:00`
//! `cargo run --target x86_64-unknown-linux-gnu -- --port /dev/ttyUSB0 run smoke`
//!

use anyhow::anyhow;
use anyhow::Ok;
use anyhow::Result;
use chrono::{Datelike, NaiveTime, TimeZone};
use chrono::{Duration as ChDuration, Utc};
use clap::{Args, Parser, Subcommand, ValueEnum};
use std::time::Duration;
use std::vec;
use std::{thread, time};
use tester::{exchange, open};
use the_protocol::SDateTime;
use the_protocol::{Command, Funct, Response};

/// Send command or suite to the device
#[derive(Parser)]
struct Cli {
    /// Serial port connected to the device
    #[arg(global = true, long)]
    port: Option<String>,

    /// Timeout
    #[arg(global = true, long, default_value_t = 1000)]
    timeout_ms: u64,

    /// Command or suite to send
    #[command(subcommand)]
    cmd: Cmd,
}

/// Commands to send
#[derive(Subcommand)]
enum Cmd {
    Reset,
    Counter,
    SetTimeNow,
    UnsetTime,
    SetDawn,
    SetNoon,
    SetEvening,
    SetNight,
    BlinkOn(PeriodArg),
    BlinkOff,
    RgbOn,
    RgbOff,
    Increment,
    Run {
        #[arg(value_enum)]
        suite: Suite,
    },
}

/// Test suites
#[derive(Copy, Clone, PartialEq, Eq, ValueEnum)]
enum Suite {
    Smoke,
    FullNoSch, // No schedule
    Full,
    Increment,
    ToggleBlink,
    ToggleRgb,
    SetTime,
    ScheduleBlink,
}

/// Time period in milliseconds
#[derive(Args)]
struct PeriodArg {
    #[arg(long, default_value_t = 300)]
    period_ms: u64,
}

/// Get the command steps from a test suite
///
/// # Arguments
/// * `suite` - Test suite
///
fn suite_steps(suite: Suite) -> Vec<Command> {
    match suite {
        // Test suite for minimum functionality
        Suite::Smoke => vec![
            Command::Reset,                                            // C1
            Command::Immediate(Funct::Increment),                      // C2
            Command::Counter,                                          // C9
            Command::Immediate(Funct::EnableBlink { period_ms: 100 }), // C3
            Command::Immediate(Funct::DisableBlink),                   // C4
        ],
        Suite::FullNoSch => vec![
            Command::Reset,
            Command::Counter,
            Command::SetDateTime(Some(Utc.timestamp(0, 0).into())),
            Command::SetDateTime(Some(Utc::now().into())),
            Command::Immediate(Funct::EnableBlink { period_ms: 100 }),
            Command::Immediate(Funct::DisableBlink),
            Command::Immediate(Funct::EnableRgb),
            Command::Immediate(Funct::DisableRgb),
        ],
        Suite::Full => vec![
            Command::Reset,
            Command::Counter,
            Command::SetDateTime(Some(Utc.timestamp(0, 0).into())),
            Command::SetDateTime(Some(Utc::now().into())),
            Command::Schedule(
                Funct::EnableBlink { period_ms: 500 },
                (Utc::now() + ChDuration::seconds(2)).into(),
            ),
            Command::Immediate(Funct::DisableBlink),
            Command::Immediate(Funct::EnableRgb),
            Command::Immediate(Funct::DisableRgb),
        ],
        Suite::Increment => vec![
            Command::Counter,
            Command::Immediate(Funct::Increment),
            Command::Counter,
        ],
        Suite::ToggleBlink => vec![
            Command::Immediate(Funct::EnableBlink { period_ms: 50 }),
            Command::Immediate(Funct::DisableBlink),
        ],
        Suite::ToggleRgb => vec![
            Command::Immediate(Funct::EnableRgb),
            Command::Immediate(Funct::DisableRgb),
        ],
        Suite::SetTime => vec![
            Command::SetDateTime(Some(Utc.timestamp(0, 0).into())),
            Command::SetDateTime(Some(Utc::now().into())),
        ],
        Suite::ScheduleBlink => vec![
            Command::SetDateTime(Some(Utc::now().into())),
            Command::Schedule(
                Funct::EnableBlink { period_ms: 50 },
                (Utc::now() + ChDuration::seconds(2)).into(),
            ),
            Command::Immediate(Funct::DisableBlink),
        ],
    }
}

/// Run the command steps
///
/// # Arguments
/// * `port` - Serial port connected to the device
/// * `steps` - Commands to send to the device
/// * `timeout` - Timeout in milliseconds, default if None
///
fn run_suite(
    port: &mut serial2::SerialPort,
    steps: &[Command],
    timeout: Option<std::time::Duration>,
) -> anyhow::Result<()> {
    let mut pass: usize = 0usize;
    let mut fail: usize = 0usize;

    // Loop through the steps
    for (i, cmd) in steps.iter().enumerate() {
        eprintln!("CASE {:02}: {:?}", i, cmd);

        // Get the result from the device
        let res: Result<Response, tester::ResponseError> = tester::exchange(cmd, port, timeout);

        match res {
            std::result::Result::Ok(resp) => {
                if resp.is_ok() {
                    println!("PASS");
                    pass += 1;
                } else {
                    println!("FAIL (resp: {:?})", resp);
                    fail += 1;
                }
            }
            std::result::Result::Err(e) => {
                println!("FAIL (io:   {:?})", e);
                fail += 1;
            }
        }

        // Add one second delay between commands
        thread::sleep(time::Duration::from_secs(1));
    }

    println!("SUMMARY: PASS {} / FAIL {}", pass, fail);
    if fail > 0 {
        return Err(anyhow!("Some steps failed"));
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    // Parse command line input
    let cli = Cli::parse();

    // Set COM_PATH variable
    if let Some(ref p) = cli.port {
        println!("Setting COM_PATH as {p:?}");
        std::env::set_var("COM_PATH", p);
    }

    // Open serial port
    println!("Opening serial port.");
    let mut port = open()?;
    let timeout = Some(Duration::from_millis(cli.timeout_ms));

    // Match the provided command
    let cmd: Command = match cli.cmd {
        Cmd::Reset => Command::Reset,
        Cmd::Counter => Command::Counter,
        Cmd::SetTimeNow => Command::SetDateTime(Some(Utc::now().into())),
        Cmd::UnsetTime => Command::SetDateTime(Some(Utc.timestamp(0, 0).into())),
        Cmd::SetDawn => {
            let today = Utc::now().date_naive();
            let naive_time = chrono::NaiveTime::from_hms_opt(4, 0, 0).expect("valid fixed time");
            let dt_utc = chrono::DateTime::<Utc>::from_utc(today.and_time(naive_time), Utc);
            Command::SetDateTime(Some(dt_utc.into()))
        }
        Cmd::SetNoon => {
            let today = Utc::now().date_naive();
            let naive_time = chrono::NaiveTime::from_hms_opt(10, 0, 0).expect("valid fixed time");
            let dt_utc = chrono::DateTime::<Utc>::from_utc(today.and_time(naive_time), Utc);
            Command::SetDateTime(Some(dt_utc.into()))
        }
        Cmd::SetEvening => {
            let today = Utc::now().date_naive();
            let naive_time = chrono::NaiveTime::from_hms_opt(16, 0, 0).expect("valid fixed time");
            let dt_utc = chrono::DateTime::<Utc>::from_utc(today.and_time(naive_time), Utc);
            Command::SetDateTime(Some(dt_utc.into()))
        }
        Cmd::SetNight => {
            let today = Utc::now().date_naive();
            let naive_time = chrono::NaiveTime::from_hms_opt(22, 0, 0).expect("valid fixed time");
            let dt_utc = chrono::DateTime::<Utc>::from_utc(today.and_time(naive_time), Utc);
            Command::SetDateTime(Some(dt_utc.into()))
        }
        Cmd::BlinkOn(a) => Command::Immediate(Funct::EnableBlink {
            period_ms: a.period_ms,
        }),
        Cmd::BlinkOff => Command::Immediate(Funct::DisableBlink),
        Cmd::RgbOn => Command::Immediate(Funct::EnableRgb),
        Cmd::RgbOff => Command::Immediate(Funct::DisableRgb),
        Cmd::Increment => Command::Immediate(Funct::Increment),
        Cmd::Run { suite } => {
            let steps = suite_steps(suite);
            return run_suite(&mut port, &steps, timeout);
        }
    };

    // Send the command
    println!("Sending command.");
    let resp =
        exchange(&cmd, &mut port, timeout).map_err(|e| anyhow!("exchange failed: {:?}", e))?;

    println!("Response: {resp:?}");
    match resp {
        Response::Rejected(r) => Err(anyhow!("Rejected: {:?}", r)),
        _ => Ok(()),
    }
}
