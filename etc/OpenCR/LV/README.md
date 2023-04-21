# Scale Truck Control System

This project is a control system for a scale truck, implemented using Arduino and ROS (Robot Operating System). The code controls the truck's throttle, steering, and gear using a PID controller. The truck's velocity and steering angle are controlled based on the received commands from the ROS node. The code also logs the data to an SD card and includes an encoder for measuring the truck's speed.

## Features

- Control of truck's throttle, steering, and transmission
- PID control for speed regulation
- Angle control using PWM
- Data logging functionality for analysis and debugging

## Hardware Requirements

- Arduino (OpenCR board)
- Servo motors for throttle, steering, and transmission
- Wheel encoder for speed measurement
- SD card for data logging

## Software Requirements

- ROS
- Arduino IDE

## Libraries Used

- `<stdio.h>`
- `<Servo.h>`
- `<math.h>`
- `<ros.h>`
- `<geometry_msgs/Twist.h>`
- `<std_msgs/Float32.h>`
- `<SD.h>`
- `<lrc2ocr.h>`
- `<ocr2lrc.h>`

## Code Explanation

### Required Libraries
The code imports the necessary libraries for various functionalities like working with servos, mathematical operations, ROS communication, reading from SD card, and handling custom messages for communication.

### Constants and Pin Configurations
The code defines several constants for timing periods, pin configurations, encoder-related parameters, and maximum and minimum values for PWM signals.

### Global Variables
The code initializes global variables, which store the current state and control values received from the ROS messages.

### Servo Objects
The code creates Servo objects for throttle, steering, and gear.

### Functions
- **LrcCallback():** A callback function that is triggered when a new message is received from the ROS topic '/lrc2ocr_msg'. It updates global variables with the received data.
- **set_gear():** This function sets the gear position using the gear Servo object.
- **setSPEED():** A PID controller function to calculate the throttle signal based on the target and current velocities, using a set of proportional, integral, and feed-forward constants.
- **setANGLE():** A function that converts the received steering angle to a PWM signal and sets the steering servo position accordingly.
- **getENA():** An interrupt service routine that reads the encoder's A and B channels to update the EN_pos_ variable, which tracks the encoder ticks.
- **CheckEN():** This function calculates the current velocity using the encoder ticks and calls the setSPEED() function to set the throttle signal based on the current and target velocities. It also logs data to the SD card.
- **ClearT():** A function to reset the encoder-related variables EN_pos_ and cumCountT_.
- **CountT():** A function called by a timer interrupt to increment the CountT_ variable.

### ROS Variables
The code initializes ROS NodeHandle, Subscriber, and Publisher objects for handling ROS communication.

### setup() and loop()
The Arduino `setup()` function initializes the ROS node, attaches servos and interrupts, initializes the SD card, and starts timer interrupts. The Arduino `loop()` function reads and processes serial inputs, spins the ROS node, and publishes the custom message at a specified interval.

## How to Set Up

1. Clone this repository to your local machine.
2. Set up the hardware components as per the requirements mentioned above.
3. Open the Arduino IDE, and configure it for the OpenCR board.
4. Compile and upload the code to the OpenCR board.
5. Launch the ROS nodes to start communicating with the control system.

## Usage

The control system listens to `/lrc2ocr_msg` topic for control commands and publishes the current state of the truck to the `/ocr2lrc_msg` topic.
