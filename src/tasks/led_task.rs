use crate::tasks::Task;
use adafruit_kb2040::hal::pio::PIOExt;
use adafruit_kb2040::hal::timer::CountDown;
use adafruit_kb2040::hal::{gpio, pio, Timer};
use adafruit_kb2040::pac;
use core::iter::once;
use core::mem;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

pub struct LedTask<'a, I, SM>
where
    I: gpio::AnyPin<Function = <pac::PIO0 as PIOExt>::PinFunction>,
    SM: pio::StateMachineIndex,
{
    timer: Timer,
    ws: Ws2812<pac::PIO0, SM, CountDown<'a>, I>,
    counter: u64,
    n: u8,
}

impl<
        'a,
        I: gpio::AnyPin<Function = <pac::PIO0 as PIOExt>::PinFunction>,
        SM: pio::StateMachineIndex,
    > LedTask<'a, I, SM>
{
    pub fn new(
        timer: Timer,
        pin: I,
        clock_freq: fugit::HertzU32,
        mut pio: &'a mut pio::PIO<pac::PIO0>,
        sm0: pio::UninitStateMachine<(pac::PIO0, SM)>,
        counter: u64,
    ) -> Self {
        let st_timer = unsafe { mem::transmute::<&Timer, &'static Timer>(&timer) };

        Self {
            timer,
            ws: Ws2812::new(pin, &mut pio, sm0, clock_freq, st_timer.count_down()),
            counter,
            n: 0,
        }
    }
}

impl<
        I: gpio::AnyPin<Function = <pac::PIO0 as PIOExt>::PinFunction>,
        SM: pio::StateMachineIndex,
    > Task for LedTask<'_, I, SM>
{
    fn run(&mut self) {
        let time = self.timer.get_counter().ticks();

        if time - self.counter >= 20000 {
            self.n = self.n.wrapping_add(1);
            let _ = self.ws.write(brightness(once(wheel(self.n)), 32));
            self.counter = time;
        }
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
