use crate::tasks::Task;
use adafruit_kb2040::hal::{
    i2c::{ValidPinScl, ValidPinSda, I2C},
    pac::I2C0,
    Timer,
};
use arrayvec::ArrayString;
use bno080::interface::I2cInterface;
use bno080::wrapper::BNO080;
use core::fmt::Write;
use core::mem;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use nalgebra::{Quaternion, Unit, UnitQuaternion, Vector3};

const PI: f32 = core::f32::consts::PI;

#[derive(Clone, Copy)]
#[repr(C, packed)]
struct HatireData {
    begin: u16,
    code: u16,
    rot: [f32; 3],
    trans: [f32; 3],
    end: u16,
}

impl HatireData {
    fn new() -> Self {
        Self {
            begin: 0xAAAA,
            code: 0,
            rot: [0.0; 3],
            trans: [0.0; 3],
            end: 0x5555,
        }
    }
}

pub struct I2cTask<'a, SDA, SCL>
    where
        SDA: ValidPinSda<I2C0>,
        SCL: ValidPinScl<I2C0>,
{
    timer: Timer,
    driver: BNO080<I2cInterface<I2C<I2C0, (SDA, SCL)>>>,
    producer: bbqueue::Producer<'a, 128>,
    accel_counter: u64,
    send_counter: u64,
    current_pos: [f32; 3],
    current_velocity: [f32; 3],
    init: bool,
}

impl<'a, SDA, SCL> I2cTask<'a, SDA, SCL>
    where
        SDA: ValidPinSda<I2C0>,
        SCL: ValidPinScl<I2C0>,
{
    pub fn new(
        timer: Timer,
        i2c: I2C<I2C0, (SDA, SCL)>,
        producer: bbqueue::Producer<'a, 128>,
        counter: u64,
    ) -> Self {
        let i2c_interface = I2cInterface::new(i2c, 0x4A);

        let driver = BNO080::new_with_interface(i2c_interface);

        Self {
            timer,
            driver,
            producer,
            accel_counter: counter,
            send_counter: counter,
            current_pos: [0.0, 0.0, 0.0],
            current_velocity: [0.0, 0.0, 0.0],
            init: false,
        }
    }

    pub fn output_string(&mut self, out: &ArrayString<128>) {
        let len = out.as_bytes().len();
        let wgr = self.producer.grant_exact(len);
        match wgr {
            Err(_) => {
                // Do nothing
            }
            Ok(mut wgr) => {
                wgr.copy_from_slice(out.as_bytes());
                wgr.commit(len);
            }
        }
    }

    pub fn output_bytes(&mut self, out: &[u8]) {
        let wgr = self.producer.grant_exact(out.len());
        match wgr {
            Err(_) => {
                // Do nothing
            }
            Ok(mut wgr) => {
                wgr.copy_from_slice(out);
                wgr.commit(out.len());
            }
        }
    }

    fn print_result<E>(&mut self, result: Result<(), E>, title: &str)
        where
            E: core::fmt::Debug,
    {
        match result {
            Err(e) => {
                let mut out_string = ArrayString::<128>::new();
                let _ = writeln!(&mut out_string, "{} Failed: {:?}", title, e);

                self.output_string(&out_string);
            }
            Ok(_) => {
                let mut out_string = ArrayString::<128>::new();
                let _ = writeln!(&mut out_string, "{} Complete", title);

                self.output_string(&out_string);
            }
        }
    }
}

impl<SDA, SCL> Task for I2cTask<'_, SDA, SCL>
    where
        SDA: ValidPinSda<I2C0>,
        SCL: ValidPinScl<I2C0>,
{
    fn run(&mut self) {
        let time = self.timer.get_counter().ticks();

        if !self.init {
            let mut delay = self.timer.count_down();
            delay.start(1000.millis());
            let _ = nb::block!(delay.wait());

            let init_res = self.driver.init(&mut self.timer);

            // self.print_result(init_res, "I2C Driver Init");

            let enable_res = self.driver.enable_gyro_integrated_rotation_vector(10u16);

            // self.print_result(enable_res, "Sensor Enable GRV Report");

            let enable_res = self.driver.enable_linear_accel(10u16);

            // self.print_result(enable_res, "Sensor Enable Linear Accel Report");

            self.init = true;
        }

        self.driver.handle_all_messages(&mut self.timer, 1u8);

        // if time - self.accel_counter >= 10000 {
        //     if let (Ok(acc), Ok(quad)) = (
        //         self.driver.linear_accel(),
        //         self.driver.rotation_quaternion(),
        //     ) {
        //         let quat = UnitQuaternion::from_quaternion(Quaternion::new(
        //             quad[0], quad[1], quad[2], quad[3],
        //         ));
        //         let quat = quat.conjugate();
        //         let acc_vec = Vector3::new(acc[0], acc[1], acc[2]);
        //         let acc_corrected = quat * acc_vec;
        //
        //         self.current_velocity[0] += acc_corrected[0] * 0.1;
        //         self.current_velocity[1] += acc_corrected[1] * 0.1;
        //         self.current_velocity[2] += acc_corrected[2] * 0.1;
        //
        //         self.current_velocity[0] /= 1.05;
        //         self.current_velocity[1] /= 1.05;
        //         self.current_velocity[2] /= 1.05;
        //
        //         self.current_pos[0] += self.current_velocity[0] * 0.01;
        //         self.current_pos[1] += self.current_velocity[1] * 0.01;
        //         self.current_pos[2] += self.current_velocity[2] * 0.01;
        //
        //         self.current_pos[0] /= 1.00005;
        //         self.current_pos[1] /= 1.00005;
        //         self.current_pos[2] /= 1.00005;
        //     }
        //
        //     self.accel_counter = time;
        // }

        if time - self.send_counter >= 10000 {
            let mut out_data = HatireData::new();

            let quad_res = self.driver.rotation_quaternion();

            if let Ok(quad) = quad_res {
                // let mut out_string = ArrayString::<128>::new();
                // let _ = writeln!(&mut out_string, "QW: {}, QX: {}, QY: {}, QZ: {}", quad[0], quad[1], quad[2], quad[3]);

                // self.output_string(&out_string);

                let quat = UnitQuaternion::from_quaternion(Quaternion::new(
                    quad[0], quad[1], quad[2], quad[3],
                ));

                let euler = quat.euler_angles();

                out_data.rot[0] = euler.0 * 180.0 / PI;
                out_data.rot[1] = euler.1 * 180.0 / PI;
                out_data.rot[2] = euler.2 * 180.0 / PI;

                // let _ = writeln!(
                //     &mut out_string,
                //     "RX: {}, RY: {}, RZ: {}",
                //     euler[0] * 180.0 / PI,
                //     euler[1] * 180.0 / PI,
                //     euler[2] * 180.0 / PI
                // );
                //
                // self.output_string(&out_string);
            }

            out_data.trans = self.current_pos.clone();

            out_data.trans[0] *= 100.0;
            out_data.trans[1] *= 100.0;
            out_data.trans[2] *= 100.0;

            self.output_bytes(&unsafe { mem::transmute::<_, [u8; 30]>(out_data) });

            self.send_counter = time;
        }
    }
}
