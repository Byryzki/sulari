# Tester

An old version of the testing software. This application can send and receive messages defined by
`the-protocol`.

Feel free to customize this program to be more useful to your purposes.

## Running programs and examples

```sh
# Send a zero byte across serial port at /dev/ttyUSB0
COM_PATH=/dev/ttyUSB0 cargo run --release --example send_zero

# Send a sequence of commands across serial port at /dev/ttyUSB0
COM_PATH=/dev/ttyUSB0 cargo run --release --example some_commands
```
