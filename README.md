# RP2040-HeadTrack

A IMU HeadTracker for use with [OpenTrack](https://github.com/opentrack/opentrack/).

- Fast and accurate 3-DOF head tracking, with pseudo 6-DOF support using numerical integration.
- 100Hz update rate


## Hardware Used

- [Adafruit KB2040](https://www.adafruit.com/product/5302)
- Adafruit 9-DOF Orientation IMU Fusion Breakout - [BNO085](https://www.adafruit.com/product/4754)
- [STEMMA QT Cable](https://www.adafruit.com/product/4210)

## Software setup

- The microcontroller opens a Serial Port through USB, and communicates with OpenTrack using the `Hatire Arduino` protocol.
