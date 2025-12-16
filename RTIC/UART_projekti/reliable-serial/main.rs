//! ## Configurations
//!
//! Pins:
//!
//! | ESP32-C3  | Host  |
//! | :-:       | :-:   |
//! | IO21/TX   | RX    |
//! | IO20/RX   | TX    |
//!
//! Baud: 115_200 BPS
#![no_std]
#![no_main]

mod serial;
const MESSAGE_BUF_SIZE: usize = 512;

// Bring in a panic handler
use panic_rtt_target as _;

#[rtic::app(device = esp32c3, dispatchers = [FROM_CPU_INTR0, FROM_CPU_INTR1, FROM_CPU_INTR2])]
mod app {
    use crate::{serial, MESSAGE_BUF_SIZE};
    use esp_hal::{
        gpio::{Level, Output, OutputConfig, Pull},
        rmt::{ConstChannelAccess, Rmt},
        time,
        uart::{self, Uart, UartRx, UartTx},
        Blocking,
    };
    use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
    use esp_println::println;
    use rtic_monotonics::esp32c3::prelude::*;
    use rtic_sync::{
        channel::{Sender, *},
        make_channel,
    };
    use rtt_target::{rprintln, rtt_init_print};
    use smart_leds::{brightness, SmartLedsWrite, RGB8};
    use the_protocol::{
        self,
        chrono::{Datelike, TimeZone, Timelike, Utc},
        Funct, RejectReason, Response, SDateTime,
    };
    use the_protocol_serde::Codec;

    // Register SysTimer as the monotonic timer for this platform
    esp32c3_systimer_monotonic!(Mono);

    #[local]
    struct Local {
        /// UART RX receives bytes which are framed into COBS packets
        uart_rx: UartRx<'static, Blocking>,
        /// RGB led for showing the time of day
        rgb_led: SmartLedsAdapter<ConstChannelAccess<esp_hal::rmt::Tx, 0>, 25>,
        led: Output<'static>,
        rgb_msg: Sender<'static, (u32, bool), 1>,
        message_buf: [u8; MESSAGE_BUF_SIZE],
        byte_counter: usize,
        // TODO: add missing local resources here as needed TODO: aggregation
        // buffer for commands which are received byte by byte
        incr: u64, // for counter data
        rgb_on: bool,
    }

    #[shared]
    struct Shared {
        /// [the_protocol_serde::Response]'s are sent back over UART TX
        uart_tx: UartTx<'static, Blocking>,
        blink_ms: u64,
        // Seconds since epoch
        reference_time: i64,
        // Used for Mono time tracking
        reference_instant: fugit::Instant<u64, 1, 16000000>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        // Retrieve the program's name from the host's environment at time of
        // compilation
        rprintln!(env!("CARGO_CRATE_NAME"));

        rprintln!("`init`: enter");

        // Set up a monotonic timer. This may be helpful for making things
        // happen on a fixed timeline
        let systimer = cx.device.SYSTIMER;
        Mono::start(systimer);

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, time::Rate::from_mhz(80u32)).unwrap();
        // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can be used
        // directly with all `smart_led` implementations
        let rmt_buffer = smart_led_buffer!(1);
        let rgb_led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO2, rmt_buffer);

        let (tx, rx) = (peripherals.GPIO21, peripherals.GPIO20);
        let mut serial = Uart::<'static>::new(
            peripherals.UART0,
            uart::Config::default().with_rx(uart::RxConfig::default().with_fifo_full_threshold(1)),
        )
        .unwrap()
        .with_rx(rx)
        .with_tx(tx);
        serial.listen(uart::UartInterrupt::RxFifoFull);

        let (uart_rx, uart_tx) = serial.split();

        let message_buf: [u8; MESSAGE_BUF_SIZE] = [0; MESSAGE_BUF_SIZE];
        let byte_counter = 0;
        rprintln!("`init`: exit");

        //blink led config
        let led_config = OutputConfig::default().with_pull(Pull::Up);
        let led = Output::new(peripherals.GPIO7, Level::Low, led_config);

        //Channel for rgb task
        let (s, r) = make_channel!((u32, bool), 1);

        let blink_ms: u64 = 0;

        let rgb_on: bool = false; // rgb off initially  need to be inited don't remove

        let reference_time: i64 = 0;

        let reference_instant: fugit::Instant<u64, 1, 16000000> = Mono::now();

        let incr: u64 = 0;

        let rgb_spawn = rgb::spawn(r);
        match rgb_spawn {
            Ok(()) => rgb_spawn.unwrap(),
            Err(_) => rprintln!("Failed to spawn matching task"),
        }

        (
            Shared {
                uart_tx: uart_tx,
                blink_ms,
                reference_time,
                reference_instant,
            },
            Local {
                uart_rx,
                rgb_led: rgb_led,
                message_buf: message_buf,
                byte_counter: byte_counter,
                led,
                rgb_msg: s.clone(),
                incr,
                rgb_on,
            },
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("`idle`: enter");
        #[allow(clippy::empty_loop)]
        loop {
            // TODO: idle task (priority = 0) may handle least urgent actions

            // Go to sleep
            // N.b., going to sleep causes a delay with RTT prints, causing output to not appear on
            // the RTT terminal. Enable with care.
            //unsafe { core::arch::asm!("wfi") };
        }
    }

    /// On UART0, aggregate incoming byte(s) to a buffer
    #[task(binds = UART0, priority = 4, local = [ uart_rx, message_buf, byte_counter ], shared = [ uart_tx ])]
    fn receive_byte(mut cx: receive_byte::Context) {
        rprintln!("`receive_byte`: enter");

        // Unpend the interrupt. This is necessary to prevent the interrupt from
        // re-firing after this task completes.
        serial::unpend_rxfifo_full_int();

        let rx = cx.local.uart_rx;
        let unit_buf = &mut [0; 1];
        while let Result::Ok(1) = rx.read_buffered(unit_buf) {
            let byte = unit_buf[0];

            // TODO: aggregate bytes, construct a command

            //Buffer overflow
            if *cx.local.byte_counter >= MESSAGE_BUF_SIZE {
                rprintln!("[Error] Message buffer overflow");
                let _tx_lock = cx.shared.uart_tx.lock(|tx| {
                    serial::send_response(
                        the_protocol::Response::Rejected(
                            the_protocol::RejectReason::CorruptedFrame,
                        ),
                        tx,
                    );
                });
                *cx.local.message_buf = [0; MESSAGE_BUF_SIZE];
                *cx.local.byte_counter = 0;
                return;
            }

            if serial::is_termination_byte(byte) {
                rprintln!("received termination byte ({})", byte);
                let slice: &mut [u8] = &mut cx.local.message_buf[0..(*cx.local.byte_counter + 1)];
                let new_command_result = the_protocol::Command::deserialize_in_place(slice);

                // Clear buffer
                *cx.local.message_buf = [0; MESSAGE_BUF_SIZE];
                *cx.local.byte_counter = 0;

                if let Err(_) = new_command_result {
                    rprintln!("[Error] Failed to deserialize command");
                    let _tx_lock = cx.shared.uart_tx.lock(|tx| {
                        serial::send_response(
                            the_protocol::Response::Rejected(
                                the_protocol::RejectReason::CorruptedFrame,
                            ),
                            tx,
                        );
                    });
                    return;
                }

                let new_command = new_command_result.unwrap();

                if cfg!(debug_assertions) {
                    rprintln!("Successfully deserialized a command.");
                }

                let new_spawn = matching::spawn(new_command);
                match new_spawn {
                    Ok(()) => new_spawn.unwrap(),
                    Err(_) => rprintln!("Failed to spawn matching task"),
                }

                return;
            } else {
                rprintln!("received byte: {}", byte);
                cx.local.message_buf[*cx.local.byte_counter] = byte;
                *cx.local.byte_counter += 1;
            }
        }
    }

    #[task(priority = 3, local = [ rgb_msg, incr, rgb_on ], shared = [ uart_tx, blink_ms, reference_time, reference_instant])]
    async fn matching(mut cx: matching::Context, comm: the_protocol::Command) {
        if cfg!(debug_assertions) {
            rprintln!("Successfully deserialized a command.");
        }

        /* Send command to corresponding task */
        match comm {
            // Requirement C1: set shared resources to init
            the_protocol::Command::Reset => {
                if cfg!(debug_assertions) {
                    rprintln!("[Debug] Received command: Reset");
                }

                *cx.local.incr = 0;

                let _blink_lock = cx.shared.blink_ms.lock(|i| {
                    *i = 0; // use task channel instead of shared resource?
                    rprintln!("blink_ms value is now {}", *i);
                });

                let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                    *t = 0;
                    rprintln!("reference_time value is now {}", *t);
                });
                //possible error in transmission handled
                match cx.local.rgb_msg.try_send((0 as u32, false)).ok() {
                    Some(m) => {
                        *cx.local.rgb_on = false;
                        m
                    }
                    None => rprintln!("Sending RGB change info to task failed."),
                }

                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                (
                    |tx|
                    {
                        serial::send_response(Response::Ok(None), tx);
                    }
                );
            }

            // Requirement C9: return shared resource increment count 'incr'
            the_protocol::Command::Counter => {
                if cfg!(debug_assertions) {
                    rprintln!("[Debug] Received command: Counter");
                }

                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                (
                    |tx|
                    {
                        serial::send_response(Response::Ok(Some(the_protocol::Payload::Counter(*cx.local.incr))), tx);
                    }
                );
            }

            // Requirement C8: set date time and follow it
            the_protocol::Command::SetDateTime(date) => {
                if cfg!(debug_assertions) {
                    rprintln!("[Debug] Received command: SetDateTime");
                }
                match date {
                    Some(time) => {
                        let dt: the_protocol::chrono::DateTime<Utc> = time.into();

                        // NOTE: Unix epoch cannot be used as the reference time, as it is considered uninitialized
                        if dt.timestamp() == 0 {
                            if *cx.local.rgb_on {
                                rprintln!("Refusing to unset DateTime since RGB led is on");
                                let _tx_lock = cx.shared.uart_tx.lock(|tx| {
                                    serial::send_response(
                                        Response::Rejected(RejectReason::IllegalCommand),
                                        tx,
                                    );
                                });
                            } else {
                                let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                                    *t = 0;
                                });

                                match cx.local.rgb_msg.try_send((0 as u32,*cx.local.rgb_on)).ok()    // update color in rgb task but keep status the same    //possible error in transmission handled
                                {
                                    Some(m) => m,
                                    None => rprintln!("Sending RGB change info to task failed."),
                                }

                                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (
                                    |tx|
                                    {
                                        serial::send_response(Response::Ok(None), tx);
                                    }
                                );
                            }
                        } else
                        // ok to set new datetime
                        {
                            let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                                rprintln!("Ref unix time was previously: {}", *t);
                                *t = dt.timestamp();
                                rprintln!("New reference time set: {}", *t);
                            });

                            let mut elapsed = 0;
                            let mut ref_time: i64 = 0;

                            let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                                ref_time = *t;
                            });

                            if ref_time == 0 {
                                let _tx_lock = cx.shared.uart_tx.lock   // respond illegal
                            (
                            |tx|
                            {
                                serial::send_response(Response::Rejected(the_protocol::RejectReason::IllegalCommand), tx);
                            }
                            );
                            }

                            let _ref_instant_lock = cx.shared.reference_instant.lock(
                                |i: &mut fugit::Instant<u64, 1, 16000000>| match Mono::now()
                                    .checked_duration_since(*i)
                                {
                                    Some(duration) => elapsed = duration.to_secs(),
                                    None => rprintln!("[Error] Failed to calculate elapsed time"),
                                },
                            );

                            let current_time_seconds = ref_time + elapsed as i64;
                            let current_dt = Utc.timestamp(current_time_seconds, 0);

                            match cx.local.rgb_msg.try_send((current_dt.hour(),*cx.local.rgb_on)).ok()    // update color in rgb task but keep status the same    //possible error in transmission handled
                            {
                                Some(m) => m,
                                None => rprintln!("Sending RGB change info to task failed."),
                            }

                            let _tx_lock = cx.shared.uart_tx.lock   // respond success
                            (
                                |tx|
                                {
                                    serial::send_response(Response::Ok(None), tx);
                                }
                            );
                        }
                    }
                    None => {
                        println!("[Error] In command SetDateTime: date is None!");

                        let _tx_lock = cx.shared.uart_tx.lock   // respond success
                        (
                            |tx|
                            {
                                serial::send_response(Response::Rejected(the_protocol::RejectReason::IllegalCommand), tx);
                            }
                        );
                    }
                }
            }

            // Requirement C2: increment shared resource increment count 'incr'
            the_protocol::Command::Immediate(func) => {
                match func {
                    the_protocol::Funct::Increment => {
                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Received command: Increment");
                        }

                        *cx.local.incr += 1;

                        let _tx_lock = cx.shared.uart_tx.lock   // respond success
                        (
                            |tx|
                            {
                                serial::send_response(Response::Ok(Some(the_protocol::Payload::Counter(*cx.local.incr))), tx);
                            }
                        );
                    }

                    the_protocol::Funct::EnableBlink { period_ms } => {
                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Received command: EnableBlink");
                        }
                        let mut is_blinking: bool = true;
                        let _blink_lock = cx.shared.blink_ms.lock(|rate| {
                            is_blinking = !(*rate == 0);
                            *rate = period_ms; // If rate = 0, turn led off
                        });

                        // Early exit to prevent double spawning, rate already got updated
                        if is_blinking {
                            if cfg!(debug_assertions) {
                                rprintln!("[Debug] Already blinking, no need to spawn");
                            }
                            let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (
                                    |tx|
                                    {
                                        serial::send_response(Response::Ok(None), tx);
                                    }
                                );
                            return;
                        }

                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Spawning new blink task");
                        }

                        let blink_spawn = blink::spawn();
                        match blink_spawn {
                            Ok(()) => {
                                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (
                                    |tx|
                                    {
                                        serial::send_response(Response::Ok(None), tx);
                                    }
                                );
                            }
                            Err(_) => {
                                if cfg!(debug_assertions) {
                                    rprintln!("[Error] Failed to spawn blink task");
                                }
                                let _tx_lock = cx.shared.uart_tx.lock   // respond error
                                (
                                    |tx|
                                    {
                                        serial::send_response(Response::Rejected(the_protocol::RejectReason::InternalError), tx);
                                    }
                                );
                            }
                        }
                    }

                    the_protocol::Funct::DisableBlink => {
                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Received command: DisableBlink");
                        }

                        let _blink_lock = cx.shared.blink_ms.lock(|rate| {
                            *rate = 0; // Turn led off by assigning rate = 0
                        });

                        let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (
                                    |tx|
                                    {
                                        serial::send_response(Response::Ok(None), tx);
                                    }
                                );
                    }

                    // Requirement C5: turn on RGB led with color corresponding to time of day
                    the_protocol::Funct::EnableRgb => {
                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Received command: EnableRgb");
                        }

                        let mut elapsed = 0;
                        let mut ref_time: i64 = 0;

                        let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                            ref_time = *t;
                        });

                        if ref_time == 0 {
                            let _tx_lock = cx.shared.uart_tx.lock   // respond illegal
                            (
                            |tx|
                            {
                                serial::send_response(Response::Rejected(the_protocol::RejectReason::IllegalCommand), tx);
                            }
                            );

                            return;
                        }

                        let _ref_instant_lock = cx.shared.reference_instant.lock(
                            |i: &mut fugit::Instant<u64, 1, 16000000>| match Mono::now()
                                .checked_duration_since(*i)
                            {
                                Some(duration) => elapsed = duration.to_secs(),
                                None => rprintln!("[Error] Failed to calculate elapsed time"),
                            },
                        );

                        let current_time_seconds = ref_time + elapsed as i64;
                        let current_dt = Utc.timestamp(current_time_seconds, 0);

                        match cx.local.rgb_msg.try_send((current_dt.hour(),true)).ok()    //possible error in transmission handled
                        {
                            Some(m) => {
                                *cx.local.rgb_on = true;
                                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (|tx| {
                                    serial::send_response(Response::Ok(None), tx);
                                });
                                m
                            },
                            None => rprintln!("Sending RGB change info to task failed."),
                        }
                    }

                    // Requirement C6: turn off RGB led
                    the_protocol::Funct::DisableRgb => {
                        if cfg!(debug_assertions) {
                            rprintln!("[Debug] Received command: DisableRgb");
                        }

                        let mut elapsed = 0;
                        let mut ref_time: i64 = 0;

                        let _ref_time_lock = cx.shared.reference_time.lock(|t| {
                            ref_time = *t;
                        });

                        if ref_time == 0 {
                            let _tx_lock = cx.shared.uart_tx.lock   // respond illegal
                            (
                            |tx|
                            {
                                serial::send_response(Response::Rejected(the_protocol::RejectReason::IllegalCommand), tx);
                            }
                            );
                        }

                        let _ref_instant_lock = cx.shared.reference_instant.lock(
                            |i: &mut fugit::Instant<u64, 1, 16000000>| match Mono::now()
                                .checked_duration_since(*i)
                            {
                                Some(duration) => elapsed = duration.to_secs(),
                                None => rprintln!("[Error] Failed to calculate elapsed time"),
                            },
                        );

                        let current_time_seconds = ref_time + elapsed as i64;
                        let current_dt = Utc.timestamp(current_time_seconds, 0);

                        match cx.local.rgb_msg.try_send((current_dt.hour(),false)).ok()    //possible error in transmission handled
                        {
                            Some(m) => {
                                *cx.local.rgb_on = false;
                                let _tx_lock = cx.shared.uart_tx.lock   // respond success
                                (|tx| {
                                    serial::send_response(Response::Ok(None), tx);
                                });
                                m
                            },
                            None => rprintln!("Sending RGB change info to task failed."),
                        }
                    }
                }
            }
            the_protocol::Command::Schedule(func, date) => {
                if cfg!(debug_assertions) {
                    rprintln!("[Debug] Received command: Schedule");
                }
            }
        }
        rprintln!("`receive_byte`: exit");
    }

    // Requirement C3 and C4: blink led in I07
    #[task(priority = 1, local = [ led ], shared = [ blink_ms, uart_tx ])]
    async fn blink(mut cx: blink::Context) {
        if cfg!(debug_assertions) {
            rprintln!("[Debug] We got in blink function!");
        }
        loop {
            let mut dur = 0;

            let _blink_lock = cx.shared.blink_ms.lock(|blink| {
                dur = *blink // Read current duration
            });

            if dur == 0 {
                cx.local.led.set_low();
                return;
            }

            cx.local.led.toggle();

            Mono::delay(dur.millis()).await;
        }
    }

    // Requirement C6 and C7: Turn RGB led on with proper color and off
    #[task(priority = 2, local = [ rgb_led ], shared = [ uart_tx, reference_time, reference_instant])]
    async fn rgb(mut cx: rgb::Context, mut next_comm: Receiver<'static, (u32, bool), 1>) {
        let mut hour = 25;
        let mut status = false;
        let mut color: RGB8 = RGB8::new(0, 0, 0);
        let mut bright: u8 = 0;

        /* channel here */
        while let Ok(val) = next_comm.recv().await
        // await for the next rgb value
        {
            (hour, status) = val;
            //Construct actual time

            match hour {
                /*Night*/
                0..=2 => {
                    color = RGB8::new(49, 8, 31);
                }
                /*Dawn*/
                3..=8 => {
                    color = RGB8::new(248, 243, 43);
                }
                /*Noon*/
                9..=14 => {
                    color = RGB8::new(156, 255, 250);
                }
                /*Evening*/
                15..=20 => {
                    color = RGB8::new(5, 60, 94);
                }
                /*Night*/
                21..=24 => {
                    color = RGB8::new(49, 8, 31);
                }
                /*Off*/
                _ => {
                    bright = 0;
                } // just in case error handling
            }

            if status {
                bright = 20;
            } else {
                bright = 0;
            }

            rprintln!("the new color is: {} and brightness is: {}", color, bright);

            rprintln!("RGB led should change now");
            cx.local
                .rgb_led
                .write(brightness(core::iter::once(color), bright))
                .unwrap(); // change RGB lde state
        }
    }
}
