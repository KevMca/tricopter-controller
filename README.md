# tricopter-controller

This project is a simple tricopter controller developed for the arduino uno. It is by no means a complex controller, however it can keep the tricopter I built relatively stable and prevent it from falling out of the sky :) The basic architecture of the controller consists of a complimentary filter on the gyro and accelerometer (this gives the orientation of the tricopter) and a separate PID controller for each axis: roll, pitch and yaw.

## Tricopter components

* Arduino uno
* Flysky FS-R6B receiver
* Flysky FS-T6 transmitter
* LSM9DS0 9-dof IMU (Adafruit Flora breakout)
* Any 9g servo
* 3x DYS BE1806-2300kv Brushless Multirotor Motor
* 3x DYS BL20A BLHeli Multirotor ESC
* Turnigy Graphene 2200mAh 3S 65C LiPo 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Required libraries:

```
#include <movingAvg.h>
#include <EnableInterrupt.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
```

If you are using a different IMU you made need other libraries to use it.

## Authors

* **Kevin McAndrew** - *Student of Electronic and computer engineering (University of Limerick)*

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Inspiration from Joop Brokking - http://www.brokking.net/ymfc-3d_main.html
