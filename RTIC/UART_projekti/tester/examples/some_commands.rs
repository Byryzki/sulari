use std::{thread, time};

use tester::{exchange, open};
use the_protocol::chrono::{self, Utc};
use the_protocol::{Command, Funct, Payload, RejectReason, Response};

fn main() {
    let mut port = open().unwrap();

    // Retrieve atomic counter
    let cmd = Command::Counter;
    let c = match exchange(&cmd, &mut port, None).unwrap() {
        Response::Ok(Some(Payload::Counter(c))) => Some(c),
        _ => {
            println!("WARN: counter was not returned by device");
            None
        }
    };

    // Increment atomic counter
    let cmd = Command::Immediate(Funct::Increment);
    exchange(&cmd, &mut port, None).unwrap();

    // Retrieve atomic counter again and check its incremented
    if let Some(c) = c {
        let cmd = Command::Counter;
        if exchange(&cmd, &mut port, None).unwrap() != Response::Ok(Some(Payload::Counter(c + 1))) {
            println!("WARN: counter value was incorrect");
        }
    }

    // Turn on now
    let cmd = Command::Immediate(Funct::EnableBlink { period_ms: 300 });
    exchange(&cmd, &mut port, None).unwrap();

    // Wait for 1 seconds
    thread::sleep(time::Duration::from_secs(1));

    // Turn off
    let cmd = Command::Immediate(Funct::DisableBlink);
    exchange(&cmd, &mut port, None).unwrap();

    // Unset date time
    let cmd = Command::SetDateTime(None);
    exchange(&cmd, &mut port, None).unwrap();

    // Turn on in 2 seconds
    let cmd = Command::Schedule(
        Funct::EnableBlink { period_ms: 500 },
        (Utc::now() + the_protocol::chrono::Duration::seconds(2)).into(),
    );
    if exchange(&cmd, &mut port, None).unwrap() != Response::Rejected(RejectReason::IllegalCommand)
    {
        println!(
            "!!! Device should have returned IllegalCommand, because reference time was not set"
        );
    }

    // Set date time
    let cmd = Command::SetDateTime(Some(Utc::now().into()));
    exchange(&cmd, &mut port, None).unwrap();

    // Turn on in 2 seconds
    let cmd = Command::Schedule(
        Funct::EnableBlink { period_ms: 500 },
        (Utc::now() + chrono::Duration::seconds(2)).into(),
    );
    if !exchange(&cmd, &mut port, None).unwrap().is_ok() {
        println!(
            "!!! Device should have returned Response::Ok, as reference time is currently set"
        );
    }

    // Wait for 3 seconds
    println!("Waiting for 3 seconds to give some time for the blink to get started");
    thread::sleep(time::Duration::from_secs(3));

    // Turn off
    let cmd = Command::Immediate(Funct::DisableBlink);
    exchange(&cmd, &mut port, None).unwrap();

    // Turn on the RGB
    let cmd = Command::Immediate(Funct::EnableRgb);
    exchange(&cmd, &mut port, None).unwrap();

    thread::sleep(time::Duration::from_secs(2));

    // Turn off the RGB
    let cmd = Command::Immediate(Funct::DisableRgb);
    exchange(&cmd, &mut port, None).unwrap();

    // Stop the blink
    let cmd = Command::Immediate(Funct::DisableBlink);
    exchange(&cmd, &mut port, None).unwrap();
}
