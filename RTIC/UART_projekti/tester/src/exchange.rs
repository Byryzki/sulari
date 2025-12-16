use std::{io, time};

use serial2::SerialPort;
use the_protocol_serde::{corncobs, Codec, Command, Response};

#[derive(Debug)]
pub enum ResponseError {
    Timeout,
}

/// Send a command over serial and wait for response from the device. Blocks until response is
/// received or timeout.
///
/// # Arguments
///
/// * `cmd` - command to send to device
/// * `port` - serial port with a connected device (ESP32-C3 serial server)
/// * `timeout` - optional timeout, default if `None`
pub fn exchange(
    cmd: &Command,
    port: &mut SerialPort,
    timeout: Option<time::Duration>,
) -> Result<Response, ResponseError> {
    send(cmd, port);
    wait_for_response(port, timeout)
}

/// Send a command over serial
pub fn send(cmd: &Command, port: &mut SerialPort) {
    println!("Serializing Command `{cmd:?}`");
    // Construct the command packet
    let mut cmd_buf = [0u8; Command::MAX_SERIALIZED_LEN];
    let cmd_packet = cmd
        .serialize(&mut cmd_buf)
        // Hard error on failing to serialize a command on the host
        .expect("Command ABI should not have changed");
    println!("Serialized packet: `{cmd_packet:?}`");

    // Send the packet over serial
    port.write(cmd_packet)
        // Hard error on failing to write over serial
        .unwrap();
}

/// Wait for a [the_protocol::Response] from the device. Blocks until response is received or timeout.
pub fn wait_for_response(
    port: &mut SerialPort,
    timeout: Option<time::Duration>,
) -> Result<Response, ResponseError> {
    const MAX_LEN: usize = Response::MAX_SERIALIZED_LEN;

    let mut resp_buf = [0u8; MAX_LEN];

    // Read byte-by-byte until we receive a packet frame
    for idx in 0..MAX_LEN {
        let byte = &mut resp_buf[idx..idx + 1];
        port.read_exact(byte).map_err(|e| match e.kind() {
            io::ErrorKind::TimedOut => ResponseError::Timeout,
            // Hard error on any other type of error
            _ => panic!("failed to read from serial: {e}"),
        })?;
        if byte[0] == corncobs::ZERO {
            break;
        }
    }

    if let Some(t) = timeout {
        port.set_read_timeout(t).unwrap();
    }
    let response = Response::deserialize_in_place(&mut resp_buf)
        // Hard error on failing to deserialize a response
        .expect("Response ABI should not have changed");
    println!("Deserialized Response: `{response:?}`");
    Ok(response)
}
