#![no_main]
#![no_std]

use panic_probe as _;
use rtic::app;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use cortex_m::singleton;
    use defmt::*;
    use stm32f1xx_hal::{
        gpio::{self, *},
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac::{self, TIM1, TIM2, TIM3, TIM4},
        prelude::*,
        serial::{Config, Serial},
        timer::{CountDownTimer, Event, Timer},
        usb::{Peripheral, UsbBus},
    };
    use rtic_monotonics::{systick::*, Monotonic};
    use usb_device::{bus::UsbBusAllocator, prelude::*};
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use heapless::{pool::Pool, spsc::Queue, Vec};
    use pca9685::{Pca9685, Channel};
    use micromath::F32Ext;
    use nb::block;

    // Constants
    const MAIN_LOOP_FREQ_HZ: u32 = 100;
    const TELEMETRY_FREQ_HZ: u32 = 2;
    const WATCHDOG_FREQ_HZ: u32 = 10;
    const USB_RX_BUFFER_SIZE: usize = 256;
    const USB_TX_BUFFER_SIZE: usize = 512;
    const ENCODER_OVERFLOW_THRESHOLD: u16 = 60000;
    const WATCHDOG_TIMEOUT_MS: u32 = 30000;

    // Shared Resources
    #[shared]
    struct Shared {
        wheel_states: [WheelState; 4],
        setpoints: [f32; 4],
        pid_controllers: [PidController; 4],
        usb_rx_queue: Queue<u8, USB_RX_BUFFER_SIZE>,
        last_cmd_timestamp: u32,
        pwm_driver: Option<Pca9685<BlockingI2c<pac::I2C1, (gpio::Pin<'B', 6, Alternate<OpenDrain>>, gpio::Pin<'B', 7, Alternate<OpenDrain>>)>>>,
    }

    // Local Resources  
    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        usb_serial: SerialPort<'static, UsbBus<Peripheral>>,
        encoder_timers: [Option<CountDownTimer<pac::TIM1>>; 4], // Will be properly typed per timer
        led: gpio::Pin<'C', 13, Output<PushPull>>,
    }

    // Global UART for debugging (unsafe but simple for testing)
    static mut UART_TX: Option<stm32f1xx_hal::serial::Tx<pac::USART1>> = None;

    // Data Structures
    #[derive(Clone, Copy, Debug)]
    struct WheelState {
        position: i32,
        velocity: f32,
        last_timestamp: u32,
        last_count: u16,
        overflow_count: i16,
    }

    #[derive(Clone, Copy, Debug)]
    struct PidController {
        kp: f32,
        ki: f32, 
        kd: f32,
        integral: f32,
        last_error: f32,
        integral_clamp: f32,
    }

    impl Default for WheelState {
        fn default() -> Self {
            Self {
                position: 0,
                velocity: 0.0,
                last_timestamp: 0,
                last_count: 0,
                overflow_count: 0,
            }
        }
    }

    impl Default for PidController {
        fn default() -> Self {
            Self {
                kp: 1.0,
                ki: 0.1,
                kd: 0.01,
                integral: 0.0,
                last_error: 0.0,
                integral_clamp: 100.0,
            }
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        info!("Mecanum Car RTIC Application Starting...");

        let mut dp = ctx.device;
        let cp = ctx.core;

        // Initialize system clock
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        // Initialize SysTick monotonic timer  
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cp.SYST, 72_000_000, systick_token);

        // Initialize GPIO
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();
        let mut afio = dp.AFIO.constrain();

        // LED pin
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // USART1 pins (PA9=TX, PA10=RX)
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10.into_floating_input(&mut gpioa.crh);
        
        // Configure USART1 (115200 baud)
        let serial = Serial::usart1(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            clocks,
        );
        let (mut tx, _rx) = serial.split();
        
        // Store TX for global access (unsafe but simple for testing)
        unsafe {
            UART_TX = Some(tx);
        }

        // Initialize I2C for PCA9685
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.Hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        // Initialize PCA9685 PWM driver
        let mut pwm_driver = Pca9685::new(i2c, 0x40).unwrap();
        // TODO: Configure PWM frequency and channels
        
        // Initialize USB
        let usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        let usb_dm = gpioa.pa11;
        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        let usb_bus = singleton!(: UsbBusAllocator<UsbBus<Peripheral>> = UsbBus::new(usb)).unwrap();
        let usb_serial = SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .product("Mecanum Car")
            .device_class(USB_CLASS_CDC)
            .build();

        // Initialize encoder timers 
        // TODO: Configure TIM1-TIM4 as encoder interfaces

        // Initialize shared resources
        let wheel_states = [WheelState::default(); 4];
        let setpoints = [0.0; 4];
        let pid_controllers = [PidController::default(); 4];
        let usb_rx_queue = Queue::new();

        // Start periodic tasks
        pid_loop::spawn().ok();
        telemetry_tx::spawn().ok();
        watchdog_task::spawn().ok();
        led_heartbeat::spawn().ok();
        
        // Send startup message
        send_debug_char('S'); // Startup
        
        info!("Initialization complete");

        (
            Shared {
                wheel_states,
                setpoints,
                pid_controllers,
                usb_rx_queue,
                last_cmd_timestamp: 0,
                pwm_driver: Some(pwm_driver),
            },
            Local {
                usb_dev,
                usb_serial,
                encoder_timers: [None, None, None, None], // TODO: Initialize properly
                led,
            },
        )
    }

    // Priority 7: Encoder ISRs
    #[task(binds = TIM1_UP, shared = [wheel_states], priority = 7)]
    fn enc0_isr(ctx: enc0_isr::Context) {
        send_debug_char('0'); // Encoder 0 ISR
        // TODO: Handle encoder 0 overflow/update interrupt
    }

    #[task(binds = TIM2, shared = [wheel_states], priority = 7)]
    fn enc1_isr(ctx: enc1_isr::Context) {
        send_debug_char('1'); // Encoder 1 ISR
        // TODO: Handle encoder 1 overflow/update interrupt  
    }

    #[task(binds = TIM3, shared = [wheel_states], priority = 7)]
    fn enc2_isr(ctx: enc2_isr::Context) {
        send_debug_char('2'); // Encoder 2 ISR
        // TODO: Handle encoder 2 overflow/update interrupt
    }

    #[task(binds = TIM4, shared = [wheel_states], priority = 7)]
    fn enc3_isr(ctx: enc3_isr::Context) {
        send_debug_char('3'); // Encoder 3 ISR
        // TODO: Handle encoder 3 overflow/update interrupt
    }

    // Priority 6: Main PID control loop at 100Hz
    #[task(shared = [wheel_states, setpoints, pid_controllers], priority = 6)]
    async fn pid_loop(ctx: pid_loop::Context) {
        let mut interval = Systick::delay(10u32.millis());
        
        loop {
            {
                let mut wheel_states = ctx.shared.wheel_states;
                let mut setpoints = ctx.shared.setpoints;
                let mut pid_controllers = ctx.shared.pid_controllers;

                (wheel_states, setpoints, pid_controllers).lock(|states, targets, pids| {
                    send_debug_char('P'); // PID loop
                    // TODO: Read current wheel velocities from encoder states
                    // TODO: Calculate PID outputs for each wheel
                    // TODO: Update PWM buffer for all 4 motors
                });
            }
            
            // Spawn PWM update task
            pwm_flush::spawn().ok();
            
            interval.next().await;
        }
    }

    // Priority 5: PWM output update
    #[task(shared = [pwm_driver], priority = 5)]
    async fn pwm_flush(ctx: pwm_flush::Context) {
        let mut pwm_driver = ctx.shared.pwm_driver;
        
        pwm_driver.lock(|driver| {
            send_debug_char('W'); // PWM Write
            if let Some(ref mut pwm) = driver {
                // TODO: Write all 4 PWM values to PCA9685 in one I2C transaction
            }
        });
    }

    // Priority 5: USB RX interrupt
    #[task(binds = USB_HP_CAN_TX, local = [usb_dev, usb_serial], shared = [usb_rx_queue], priority = 5)]
    fn usb_rx_isr(ctx: usb_rx_isr::Context) {
        let usb_dev = ctx.local.usb_dev;
        let usb_serial = ctx.local.usb_serial;
        let mut rx_queue = ctx.shared.usb_rx_queue;

        send_debug_char('U'); // USB RX
        if usb_dev.poll(&mut [usb_serial]) {
            let mut buf = [0u8; 64];
            match usb_serial.read(&mut buf) {
                Ok(count) => {
                    rx_queue.lock(|queue| {
                        for i in 0..count {
                            // TODO: Push bytes to ring buffer
                            let _ = queue.enqueue(buf[i]);
                        }
                    });
                    
                    // Spawn command parser if we have data
                    cmd_parser::spawn().ok();
                }
                Err(_) => {
                    // TODO: Handle USB read error
                }
            }
        }
    }

    // Priority 4: Command parsing and protocol handling  
    #[task(shared = [usb_rx_queue, setpoints, pid_controllers, last_cmd_timestamp], priority = 4)]
    async fn cmd_parser(ctx: cmd_parser::Context) {
        let mut rx_queue = ctx.shared.usb_rx_queue;
        let mut setpoints = ctx.shared.setpoints;
        let mut pid_controllers = ctx.shared.pid_controllers;
        let mut last_cmd_timestamp = ctx.shared.last_cmd_timestamp;

        send_debug_char('C'); // Command parser
        // TODO: Parse incoming USB commands
        // TODO: Handle VEL, SET_PID, and other protocol commands
        // TODO: Update setpoints and PID parameters
        // TODO: Update command timestamp for watchdog
    }

    // Priority 3: Telemetry transmission at 2Hz
    #[task(local = [usb_serial], shared = [wheel_states, setpoints], priority = 3)]
    async fn telemetry_tx(ctx: telemetry_tx::Context) {
        let mut interval = Systick::delay(500u32.millis());
        
        loop {
            {
                let wheel_states = ctx.shared.wheel_states;
                let setpoints = ctx.shared.setpoints;

                (wheel_states, setpoints).lock(|states, targets| {
                    send_debug_char('T'); // Telemetry
                    // TODO: Format telemetry data
                    // TODO: Send status frame via USB
                });
            }
            
            interval.next().await;
        }
    }

    // Priority 2: Watchdog safety task at 10Hz
    #[task(shared = [last_cmd_timestamp, setpoints], priority = 2)]
    async fn watchdog_task(ctx: watchdog_task::Context) {
        let mut interval = Systick::delay(100u32.millis());
        
        loop {
            {
                let last_cmd_timestamp = ctx.shared.last_cmd_timestamp;
                let mut setpoints = ctx.shared.setpoints;

                (last_cmd_timestamp, setpoints).lock(|last_ts, targets| {
                    send_debug_char('D'); // Watchdog
                    let now = Systick::now().duration_since_epoch().to_millis() as u32;
                    
                    if now.saturating_sub(*last_ts) > WATCHDOG_TIMEOUT_MS {
                        // TODO: Emergency stop - set all setpoints to 0
                        for target in targets.iter_mut() {
                            *target = 0.0;
                        }
                    }
                });
            }
            
            interval.next().await;
        }
    }

    // Priority 1: LED heartbeat at 2Hz  
    #[task(local = [led], priority = 1)]
    async fn led_heartbeat(ctx: led_heartbeat::Context) {
        let mut interval = Systick::delay(500u32.millis());
        let led = ctx.local.led;
        
        loop {
            send_debug_char('H'); // Heartbeat
            led.toggle();
            interval.next().await;
        }
    }

    // Helper functions (to be implemented)
    fn calculate_wheel_velocity(state: &mut WheelState, current_count: u16, timestamp: u32) -> f32 {
        // TODO: Calculate velocity from encoder ticks and time delta
        0.0
    }

    fn pid_update(controller: &mut PidController, setpoint: f32, measured: f32, dt: f32) -> f32 {
        // TODO: Implement PID controller algorithm
        0.0
    }

    fn parse_command(buffer: &[u8]) -> Option<Command> {
        // TODO: Parse ASCII protocol commands
        None
    }

    fn format_telemetry(states: &[WheelState; 4], setpoints: &[f32; 4]) -> Vec<u8, 256> {
        // TODO: Format telemetry data for transmission
        Vec::new()
    }

    // UART debug output function
    fn send_debug_char(c: char) {
        unsafe {
            if let Some(ref mut tx) = UART_TX {
                let _ = block!(tx.write(c as u8));
            }
        }
    }

    #[derive(Debug)]
    enum Command {
        Velocity([f32; 4]),
        SetPid(usize, f32, f32, f32),
        Stop,
    }
}