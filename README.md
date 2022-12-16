# IMUnipulator

This project is one-half of the **IMUGripulator**, a gesture/EMG-controlled 2-DOF robotic arm and gripper. The robot was created as a collaboration between two teams for the final project of Northwestern University's CE 346 Microprocessor System Design course. Each team used a [micro:bit v2](https://tech.microbit.org/hardware/2-0-revision/) with a [Nordic nRF52833 SoC](https://www.nordicsemi.com/Products/nRF52833) to create an application in C to control their portion of the robot.

The **IMUnipulator** team used a 9-DOF IMU to control two servos that actuated the joints of the robotic arm. The code for this application is in this repository, [here](software/apps/final_project/).

The **EMGripper** team used an EMG sensor and a force sensitive resistor to control a servo that actuated the arm's end-effector gripper. Their code can be found in a [separate repository](https://github.com/katie-hughes/emgripper).

## Video

[PickingUpTux](https://user-images.githubusercontent.com/113186159/208053732-0d4c12c5-b25e-4fb4-a2f9-99a4de32a25b.mp4)

## Team
- Felipe Jannarone
- [Nick Morales](https://github.com/ngmor)
- [Hang-Yin](https://github.com/hang-yin)

## Sensors

- [Adafruit TDK InvenSense ICM-20948 9-DOF IMU](https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu) - provided pitch and roll signals from the accelerometer to control the arm joints.
- [SparkFun AT42QT1011 Capacitive Touch Breakout](https://www.sparkfun.com/products/14520) (x2) - buttons for resetting the arm to home position and controlling an electromagnet end-effector.
- Built-in micro:bit Capacitive Touch Sensor - used to adjust the sensitivity level (speed) at which the arm's joints moved in response to control signals from the IMU.

## Actuators
- MG996R and DS3218 servos - used to move the arm joints in response to the IMU signals.
- [Adafruit PCA9685 I2C 16-Channel 12-bit PWM Driver](https://www.adafruit.com/product/815) - used as an I2C interface to send PWM signals to the servos.
- [Adafruit 5V Electromagnet](https://www.adafruit.com/product/3872) - an optional end-effector for the arm to pick up metallic objects.
- Built-in micro:bit LED Matrix - used to indicate the currently active sensitivity level (speed) for arm motion

## Control Overview
The micro:bit collects roll and pitch data via I2C from the IMU's accelerometer and converts the data into degrees of tilt in each axis. If either axis indicates a tilt that is higher than a certain threshold, the corresponding servo will be commanded to move. The roll axis controls the lower joint of the arm, called the "base" joint. The pitch axis controls the upper joint of the arm, called the "arm" joint. To simplify user experience, only one axis can move at a time, and the arm joint is prioritized over the base joint.

The servos are controlled via PWM duty cycles, each one indicating a different position to which the servo should move. The micro:bit sends data over I2C to set the value of that duty cycle to the PCA9685, which in turn produces a PWM signal with that duty cycle. Each servo has a mechanical limit of 180 degrees of rotation, though the arm joint is limited in software to less than that.

While the IMU axis remains tilted beyond its threshold, the servo commanded position is incremented/decremented at a certain rate. This rate can be controlled by the "sensitivity" level of the arm, which is adjusted via the micro:bit's built-in capacitive touch sensor and indicated by the built-in LED matrix.

Finally, the arm can be returned to its default "home" position via a touch of an external capacitive touch sensor.