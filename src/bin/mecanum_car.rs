#![no_main]
#![no_std]

use small_stm32_rust_car as _; // global logger + panicking-behavior + memory layout
use rtic::app;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3])]
mod app {
    use stm32f1xx_hal::{
        gpio::{self, *},
        pac,
        prelude::*,
        usb::{Peripheral, UsbBus},
    };
    use usb_device::prelude::*;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use cortex_m::asm::delay;

    // Constants
    const MAIN_LOOP_FREQ_HZ: u32 = 100;
    const TELEMETRY_FREQ_HZ: u32 = 2;
    const WATCHDOG_FREQ_HZ: u32 = 10;

    // Shared Resources
    #[shared]
    struct Shared {
        counter: u32,
    }

    // Local Resources  
    #[local]
    struct Local {
        led: gpio::Pin<'C', 13, Output<PushPull>>,
    }

    // Global USB serial for debugging (unsafe but simple for testing)
    static mut USB_SERIAL: Option<SerialPort<UsbBus<Peripheral>>> = None;
    static mut USB_DEVICE: Option<UsbDevice<UsbBus<Peripheral>>> = None;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Mecanum Car RTIC Application Starting...");

        let dp = ctx.device;
        let _cp = ctx.core;

        // Initialize system clock (48MHz for USB)
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);
        
        assert!(clocks.usbclk_valid());

        // Initialize SysTick monotonic timer - not needed for simplified version
        // let mono = Systick::new(cp.SYST, 72_000_000);

        // Initialize GPIO
        let mut gpioa = dp.GPIOA.split();
        let mut gpioc = dp.GPIOC.split();

        // LED pin
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // USART1 pins (PA9=TX, PA10=RX)
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10.into_floating_input(&mut gpioa.crh);
        
        // Configure USART1 (115200 baud)
        let serial = Serial::new(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            &clocks,
        );
        let (tx, _rx) = serial.split();
        
        // Store TX for global access (unsafe but simple for testing)
        unsafe {
            UART_TX = Some(tx);
        }

        // Start periodic tasks
        pid_loop::spawn().ok();
        telemetry_tx::spawn().ok();
        watchdog_task::spawn().ok();
        led_heartbeat::spawn().ok();
        
        // Send startup message
        send_debug_char('S'); // Startup
        
        defmt::info!("Initialization complete");

        (
            Shared {
                counter: 0,
            },
            Local {
                led,
            },
        )
    }

    // Priority 6: Main PID control loop at 100Hz
    #[task(shared = [counter], priority = 6)]
    async fn pid_loop(mut ctx: pid_loop::Context) {
        ctx.shared.counter.lock(|_counter| {
            *_counter += 1;
        });
        send_debug_char('P'); // PID loop
        
        // Schedule next execution (simplified - no timing)
        pid_loop::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // Priority 3: Telemetry transmission at 2Hz
    #[task(shared = [counter], priority = 3)]
    async fn telemetry_tx(mut ctx: telemetry_tx::Context) {
        ctx.shared.counter.lock(|_counter| {
            send_debug_char('T'); // Telemetry
            // Could send counter value here
        });
        
        // Schedule next execution (simplified - no timing)
        telemetry_tx::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // Priority 2: Watchdog safety task at 10Hz
    #[task(shared = [counter], priority = 2)]
    async fn watchdog_task(mut ctx: watchdog_task::Context) {
        ctx.shared.counter.lock(|_counter| {
            send_debug_char('D'); // Watchdog
            // Watchdog logic here
        });
        
        // Schedule next execution (simplified - no timing)
        watchdog_task::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // Priority 1: LED heartbeat at 2Hz  
    #[task(local = [led], priority = 1)]
    async fn led_heartbeat(ctx: led_heartbeat::Context) {
        send_debug_char('H'); // Heartbeat
        ctx.local.led.toggle();
        
        // Schedule next execution (simplified - no timing)
        led_heartbeat::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // USB serial debug output function
    fn send_debug_char(c: char) {
        unsafe {
            if let Some(ref mut serial) = USB_SERIAL {
                if let Some(ref mut usb_dev) = USB_DEVICE {
                    let _ = serial.write(&[c as u8]);
                    usb_dev.poll(&mut [serial]);
                }
            }
        }
    }
}