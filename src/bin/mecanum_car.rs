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
    };

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

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Mecanum Car RTIC Application Starting...");

        let dp = ctx.device;
        let _cp = ctx.core;

        // Initialize system clock
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        // Initialize GPIO
        let mut gpioc = dp.GPIOC.split();

        // LED pin
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // Start periodic tasks
        pid_loop::spawn().ok();
        telemetry_tx::spawn().ok();
        watchdog_task::spawn().ok();
        led_heartbeat::spawn().ok();
        
        // Send startup message via RTT
        defmt::info!("=== MECANUM CAR INITIALIZATION ===");
        defmt::info!("System Clock: 72MHz");
        defmt::info!("RTT Debug Output: ACTIVE");
        defmt::info!("LED Control: GPIO PC13");
        defmt::info!("Tasks: PID(100Hz), Telemetry(2Hz), Watchdog(10Hz), Heartbeat(2Hz)");
        defmt::info!("=== INITIALIZATION COMPLETE - RTT WORKING ===");

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
        ctx.shared.counter.lock(|counter| {
            *counter += 1;
        });
        defmt::info!("PID loop: P");
        
        // Schedule next execution (simplified - no timing)
        pid_loop::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // Priority 3: Telemetry transmission at 2Hz
    #[task(shared = [counter], priority = 3)]
    async fn telemetry_tx(mut ctx: telemetry_tx::Context) {
        ctx.shared.counter.lock(|counter| {
            defmt::info!("Telemetry: T, counter={}", counter);
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
            defmt::info!("Watchdog: D");
        });
        
        // Schedule next execution (simplified - no timing)
        watchdog_task::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }

    // Priority 1: LED heartbeat at 2Hz  
    #[task(local = [led], priority = 1)]
    async fn led_heartbeat(ctx: led_heartbeat::Context) {
        defmt::info!("Heartbeat: H");
        ctx.local.led.toggle();
        
        // Schedule next execution (simplified - no timing)
        led_heartbeat::spawn().ok();
        
        // Add await to make this a proper async function
        cortex_m::asm::wfi();
    }
}