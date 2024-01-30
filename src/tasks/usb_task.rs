use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use crate::tasks::Task;
use adafruit_kb2040::hal::{timer::Timer, usb};

pub struct UsbTask<'a> {
    timer: Timer,
    usb_dev: UsbDevice<'a, usb::UsbBus>,
    serial: SerialPort<'a, usb::UsbBus>,
    consumer: bbqueue::Consumer<'a, 128>,
}

impl<'a> UsbTask<'a> {
    pub(crate) fn new(
        timer: Timer,
        usb_bus: &'a UsbBusAllocator<usb::UsbBus>,
        consumer: bbqueue::Consumer<'a, 128>,
    ) -> Self {
        let serial = SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c2, 0x27de))
            .manufacturer("NELC")
            .product("KB2040-HEADTRACK")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        Self {
            timer,
            usb_dev,
            serial,
            consumer,
        }
    }
}

impl Task for UsbTask<'_> {
    fn run(&mut self) {
        let time = self.timer.get_counter().ticks();

        if self.serial.dtr() {
            let rgr = self.consumer.read();

            match rgr {
                Err(_) => {
                    // Do nothing
                }
                Ok(rgr) => {
                    let len = rgr.len();

                    if len > 0 {
                        let _ = self.serial.write(&rgr[0..len]);

                        rgr.release(len);
                    }
                }
            }
        }

        // Demo function, unused
        if self.usb_dev.poll(&mut [&mut self.serial]) {
            let mut buf = [0u8; 64];
            match self.serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(_) => {
                    // Do nothing
                }
            }
        }
    }
}
