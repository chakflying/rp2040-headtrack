pub mod i2c_task;
pub mod led_task;
pub mod usb_task;

pub trait Task {
    fn run(&mut self);
}
