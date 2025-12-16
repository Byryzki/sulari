//! Sends a zero (frame-byte) to the device
//!
//! The serial terminal can be specified using the `COM_PATH` environment
//! variable, e.g.,
//!
//! ```sh
//! export COM_PATH=/dev/ttyUSB0
//! ```
use tester::open;

fn main() {
    let port = open().unwrap();

    println!("Sending zero...");
    port.write(&[the_protocol_serde::corncobs::ZERO]).unwrap();
    println!("done");
}
