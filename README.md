Project Overview

This Arduino-based project involves creating a versatile robot capable of following a predefined line, avoiding obstacles in its path, and being controlled via Bluetooth communication. The robot incorporates a servo motor for positioning the ultrasonic sensor, a line-detecting sensor, and is built with an Arduino board for its brains.


Features

Line Following: The robot is equipped with sensors to detect and follow a specified line on the ground.
Obstacle Avoidance: It can detect obstacles in its path using ultrasonic sensors and autonomously navigate around them.
Bluetooth Control: The robot can be controlled remotely using a Bluetooth module, enabling users to send commands and adjust its behavior.
Servo Motor for Ultrasonic Sensor: A servo motor is used to position the ultrasonic sensor, allowing the robot to scan its surroundings for   obstacles.
Line Detecting Sensor: The robot incorporates a line-detecting sensor to follow a predefined path.


Hardware Requirements

Arduino Board (e.g., Arduino Uno)
Motor Driver (e.g., L298N)
Infrared (IR) Sensors for Line Following
Ultrasonic Sensor for Obstacle Avoidance
Servo Motor for Ultrasonic Sensor positioning
Line Detecting Sensor (e.g., IR Reflectance Sensor)
Bluetooth Module (e.g., HC-05)
Chassis and Motors
Power Supply (Battery or Power Bank)
Jumper Wires
Breadboard
Software Requirements
Arduino IDE (https://www.arduino.cc/en/software)
Bluetooth Terminal App (e.g., Arduino Bluetooth Controller)


Wiring Instructions

Connect IR sensors to detect the line.
Connect the line-detecting sensor for precise line following.
Connect the ultrasonic sensor to detect obstacles.
Connect the servo motor to control the ultrasonic sensor's position.
Connect the motors to the motor driver for driving the robot.
Connect the Bluetooth module for the remote control.
Refer to the provided circuit diagram for detailed wiring instructions.

Arduino Code
The Arduino code (finalcode.ino) is included in this repository. Upload the code to your Arduino board using the Arduino IDE.


Usage

Power up the robot using the appropriate power source.
Pair your Bluetooth-enabled device with the HC-05 module.
Open the Bluetooth Terminal App and connect to the HC-05 module.
Send commands through the Bluetooth Terminal to control the robot manually.


Notes

Ensure proper calibration of sensors for optimal performance.
Customize the code to adjust the robot's behavior and responsiveness.
Experiment with different line-following algorithms for improved accuracy.

