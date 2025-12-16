//! Methods for controlling the serial port
#![allow(unused)]
use esp_hal::{uart::UartTx, Blocking};
use the_protocol_serde::{corncobs::ZERO, Codec, Response};

/// Sends a [the_protocol::Response] over provided UART
pub fn send_response(resp: Response, tx: &mut UartTx<'static, Blocking>) {
    let mut out_buf = [0u8; Response::MAX_SERIALIZED_LEN];
    uart_write(
        resp.serialize(&mut out_buf)
            // There is no way to recover from this, nor should it ever fail
            .expect("unable to serialize response"),
        tx,
    );
}

/// Writes a buffer of bytes over provided UART
pub fn uart_write(buf: &[u8], uart_tx: &mut UartTx<'static, Blocking>) {
    // Write as long as more than 0 bytes remain to be written
    let mut rem = buf.len();
    while rem > 0 {
        rem -= uart_tx.write(buf).unwrap();
    }
}

/// Clears the interrupt for RX FIFO full, preventing the interrupt from re-firing
///
/// This is a workaround for esp-hal 0.20.1 <= 1.0.0-rc.0+ which are missing the
/// appropriate API on [esp_hal::uart::UartRx] for unpending this particular interrupt
pub(crate) fn unpend_rxfifo_full_int() {
    let uart0 = unsafe { esp32c3::UART0::steal() };
    uart0.int_clr().write(|w| w.rxfifo_full().bit(true));
}

/// Clears the interrupt for RX FIFO full, preventing the interrupt from re-firing
///
/// This is a workaround for esp-hal 0.20.1 <= 1.0.0-rc.0+ which are missing the
/// appropriate API on [esp_hal::uart::UartRx] for unpending this particular interrupt
pub(crate) fn unpend_at_cmd_char_det_int() {
    let uart0 = unsafe { esp32c3::UART0::steal() };
    uart0.int_clr().write(|w| w.at_cmd_char_det().bit(true));
}

pub(crate) fn is_termination_byte(byte: u8) -> bool {
    byte == ZERO
}
