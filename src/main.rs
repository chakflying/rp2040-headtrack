#![no_std]
#![no_main]

mod tasks;

/// Ensure we halt the program on panic (if we don't mention this crate it won't
/// be linked)
use panic_halt as _;
use usb_device::class_prelude::*;

use crate::tasks::i2c_task::I2cTask;
use adafruit_kb2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        i2c,
        multicore::{Multicore, Stack},
        pac,
        pio::PIOExt,
        timer::Timer,
        usb,
        watchdog::Watchdog,
        Sio,
    },
    XOSC_CRYSTAL_FREQ,
};
use bbqueue::BBBuffer;
use fugit::RateExtU32;

use crate::tasks::led_task::LedTask;
use crate::tasks::usb_task::UsbTask;
use crate::tasks::Task;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything seperately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

static WRITE_QUEUE: BBBuffer<128> = BBBuffer::new();

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this
/// function as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then the LED, then runs
/// the colour wheel in an infinite loop.
#[adafruit_kb2040::entry]
fn main() -> ! {
    // Configure the RP2040 peripherals

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let (prod, cons) = WRITE_QUEUE.try_split().unwrap();

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut sio = Sio::new(pac.SIO);

    let pins = adafruit_kb2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Setup USB
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Setup I2C
    let i2c = i2c::I2C::i2c0(
        pac.I2C0,
        pins.sda.into_function(),
        pins.scl.into_function(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Setup Multicore
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let mut led_task = LedTask::new(
            timer.clone(),
            pins.neopixel.into_function(),
            clocks.peripheral_clock.freq(),
            &mut pio,
            sm0,
            timer.get_counter().ticks(),
        );

        let mut i2c_task = I2cTask::new(timer.clone(), i2c, prod, timer.get_counter().ticks());

        loop {
            // led_task.run();
            i2c_task.run();
        }
    });

    let mut usb_task = UsbTask::new(timer.clone(), &usb_bus, cons);

    loop {
        usb_task.run();
    }
}
