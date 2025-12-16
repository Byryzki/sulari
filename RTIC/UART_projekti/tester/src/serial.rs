use std::{env, io, sync, time::Duration};

use serial2::SerialPort;

/// File path to serial terminal, e.g., "/dev/ttyUSB0". Can be specified using the `COM_PATH`
/// environment variable.
static COM_PATH: sync::LazyLock<String> = sync::LazyLock::new(|| {
    env::var("COM_PATH").ok().unwrap_or_else(|| {
        let default_path = if cfg!(target_os = "linux") {
            Some("/dev/ttyUSB0")
        }
        // On Windows, use something like "COM1". For COM ports above COM9, you need to use
        // the win32 device namespace, for example "\\.\COM10" (or "\\\\.\\COM10" with
        // string escaping). For more details, see:
        // https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-file?redirectedfrom=MSDN#win32-device-namespaces
        else if cfg!(target_os = "windows") {
            Some("COM3")
        }
        // I have no idea what they use on Mac or any other platform
        else {
            None
        };
        if let Some(p) = default_path {
            println!("COM_PATH not provided via env, using platform default: {p}");
            p.to_string()
        } else {
            panic!("Please specify COM_PATH via env")
        }
    })
});

// One second timeout for getting a response from the device
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(1000);

/// Opens a serial port
pub fn open() -> io::Result<SerialPort> {
    let mut port = SerialPort::open(&*COM_PATH, 115200)?;

    // Needed for windows, but should not hurt on Linux
    port.set_dtr(true)?;
    port.set_rts(true)?;
    port.set_write_timeout(DEFAULT_TIMEOUT)?;
    port.set_read_timeout(DEFAULT_TIMEOUT)?;

    Ok(port)
}
