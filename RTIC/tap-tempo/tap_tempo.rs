
#![no_std]
#![no_main]

// Bring in panic handler
use panic_rtt_target as _;

#[rtic::app(device = esp32c3, dispatchers=[FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {

    use heapless::Deque;

    use rtt_target::{rprintln, rtt_init_print};

    use rtic_monotonics::{esp32c3::prelude::*, esp32c3_systimer_monotonic};
    esp32c3_systimer_monotonic!(Mono);

    use rtic_sync::{channel::*, make_channel};
    
    use esp_hal::{
        self as _, gpio::{Event, Input, InputConfig, Output, OutputConfig, Pull, Level}, riscv::asm::nop
    };

    #[shared]
    struct Shared {
        delay_buffer: Deque::<fugit::Duration<u64, 1, 16000000>, 6>,
    }

    #[local]
    struct Local {
        button: Input<'static>,
        message: Sender<'static, fugit::Instant<u64, 1, 16000000>, BUF_SIZE>,
        prev_moment: fugit::Instant<u64, 1, 16000000>,
        led: Output<'static>,
    }

    // Global constants
    const BUF_SIZE: usize = 7;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        rtt_init_print!();
        rprintln!(env!("CARGO_CRATE_NAME"));

        let peripherals = esp_hal::init(esp_hal::Config::default());

        //button configs
        let button_config = InputConfig::default().with_pull(Pull::Up);
        let mut button = Input::new(peripherals.GPIO9, button_config);
        button.listen(Event::AnyEdge);

        //led config
        let led_config = OutputConfig::default().with_pull(Pull::Up);
        let led = Output::new(peripherals.GPIO7, Level::Low, led_config);

        //timer start and first time increment
        Mono::start(cx.core.SYSTIMER);
        let prev_moment: fugit::Instant<u64, 1, 16000000> = Mono::now();
 
        //Channel for comms from HW interrupts
        let (s, r) = make_channel!(fugit::Instant<u64, 1, 16000000>, BUF_SIZE);  
        
        //Spawn channel
        let spawn_buffer: Result<(), _> = buffer_handler::spawn(r); //unwrap checked for erronous spawn
        match spawn_buffer
        {
            Ok(()) => spawn_buffer.unwrap(),
            Err(_) => rprintln!("Spawning buffer_handler failed :/"),
        }

        //Heapless queue to buffer the delays
        let delay_buffer = Deque::<fugit::Duration<u64, 1, 16000000>, 6>::new();

        (Shared { delay_buffer }, Local { button, message: s.clone(), prev_moment, led })
    }

    #[idle]
    fn idle(_: idle::Context) -> !  // idle for waiting the first interrupt and other idling
    {
        //rprintln!("Hello from eepy idle!");

        loop    //idle waiting mode
            {
                nop();   
            }
    }

    // Interrupt for button pressed and button released counting the time increments

    #[task(priority = 3, binds = GPIO, local = [button, message])]
    fn button(cx: button::Context)
    {
        
        let moment: fugit::Instant<u64, 1, 16000000> = Mono::now();
        
        //rprintln!("[{}] button changed\n", moment);
        cx.local.button.clear_interrupt();

        match cx.local.message.try_send(moment).ok()    //possible error in transmission handled
        {
            Some(m) => m,
            None => rprintln!("Sending time increment from button interrupt failed."),
        }

    }

    // Software interrupt to handle, start and control the LED sequence when buffer is full / three button presses and three releases recorded
    #[task(priority = 2, local = [prev_moment], shared = [delay_buffer])]
    async fn buffer_handler(mut cx: buffer_handler::Context, mut incomming: Receiver<'static, fugit::Instant<u64, 1, 16000000>, BUF_SIZE>)
    {
        let mut start_leds: bool = false;
        
        while let Ok(moment) = incomming.recv().await
        {
            let mut duration: fugit::Duration<u64, 1, 16000000> = moment.checked_duration_since(*cx.local.prev_moment)
            .unwrap_or(fugit::Duration::<u64, 1, 16000000>::from_ticks(0)); // Panicing prevented with _or condition
            
            *cx.local.prev_moment = Mono::now();

            if duration > fugit::Duration::<u64, 1, 16000000>::from_ticks(2*16000000)   // Cap the maximum delay to 2 seconds
            {
                duration = fugit::Duration::<u64, 1, 16000000>::from_ticks(2*16000000);
            }

            let _locked_buf = cx.shared.delay_buffer.lock
            (
                |buf|
                {
                    match buf.push_front(duration)
                    {
                        Ok(()) => {},
                        Err(_value_back) =>  // Buffer must be full
                        {
                            buf.pop_back(); //remove the oldest element
                            unsafe{buf.push_front_unchecked(duration)};    // SAFETY: Due to first popping the back in mutex cover, no chance of overflow.
                        },
                    }
                    //rprintln!("Buffer status: {}\n", buf.len());

                    if buf.is_full(){start_leds = true}
                }
            );

            if start_leds
            {
                let spawn: Result<(), _> = led_sequence::spawn();   // Unwrap checked for erronous spawn
                match spawn
                {
                    Ok(()) => spawn.unwrap(),
                    Err(_) => rprintln!("led_sequence already spawned"),
                }
                
            }

        }
    }

    // spawn when we have 3 presses to enable the correct cadence for the LEDs
    #[task(priority = 1, local = [led], shared = [delay_buffer])]   
    async fn led_sequence(mut cx: led_sequence::Context)
    {
        let mut delays: Deque::<fugit::Duration<u64, 1, 16000000>, 6> = Deque::new();
        
        loop
        {
            // buffer is checked once per led sequence for new delays
            let _locked_buf = cx.shared.delay_buffer.lock
            (
                |buf|
                {
                    delays = buf.clone();
                    
                }
            );
            
            //Blink the led according to delays
            for delay in &delays
            {
                cx.local.led.toggle();
                //rprintln!("Delay duration: {}\n", delay);
                Mono::delay(*delay).await;
            }
        }
        
    }
    
}
