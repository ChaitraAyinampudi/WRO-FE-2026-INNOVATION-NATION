# WRO 2026 FUTURE ENGINEERS: INNOVATION NATION
Welcome to the WRO Future Engineers 2026 **Innovation Nation** Documentation! Here you will find all the documentation, source code, images, and models related to our autonomous robot.

(Innovation Nation Photo or sm)

## Our Team: 

Team Innovation Nation: Kyle Ho, Chaitra Ayinampudi, and Narasimha Yalamanchi
(Put more abt team)

(IMAGE of us)

## Project Overview: 

Stuff

## Mobility & Mechanical Design

Stuff

### Our Robot:

IMAGE of the robot 

**Dimensions:** 

(put the width x height of our robot)

**Weight:** 

(Put the weight of our robot)

**Torque/speed:**
Our top speed is 0.11Km/h (1.9 meters per minute)

**Design trade-offs:**

### Materials:

##### 2 battery boxes (stores 2 each): 
The battery holders and batteries are our power source for the robot; our holder has a switch to easily turn our robot on or off. Easy to take off or put on our robot.

##### 4 TAKEN 18 x 67mm 3.7V lithium Rechargeable batteries: 
Our robot inputs 16 Volts of power. Rechargeable batteries for long-term use.

##### DC/DC Converter:
DC/DC Converter to give power to our Raspberry Pi, U2D2 controller, and Pi Pico.

<img width="250" height="200" alt="Screenshot 2026-08-02 144234" src="https://github.com/user-attachments/assets/75fcd9cf-2528-4111-bc38-31dceb744f4a" />

##### U2D2 Controller:
U2D2 Controller so that our Raspberry Pi can control our DYNAMIXEL motors

##### 2 DYNAMIXEL XL330-M288-T Motors:
XL330-M288-T motors since it's a fast, reliable digital motor

##### Raspberry Pi 4 Model B:
Raspberry Pi 4 Model B since it's a reliable and cost-effective computer.

##### Raspberry Pi Sense HAT:
Sense HAT to use their LED to print which model will be run during rounds; the joystick is used to select and run a model.

##### Raspberry Pi Pico:
Pi Pico to control our ToF sensors and IMU Sensor without using the Raspberry Pi too much.

##### 4 VL53L0X ToF Sensors:
The ToF Sensors calculate the distance between the robot, wall, and obstacles. We placed one in the front, one in the back, and two on the right side of the robot. We placed two on the right side to make the robot's distance and rotation (angle) more accurate. We also used ToF sensors as distance sensors for parking and finding the parking boundaries

##### WT901 9-axis IMU Sensor:
IMU sensor for reliable readings of how many laps we ran; 360 degrees is one lap.

##### Raspberry Pi Camera Module v1 / OV5647:
Pi Camera to train and run our AI models.

##### USB to MicroUSB Wire
USB to microUSB wire to have our Raspberry Pi control our Pi Pico.

##### USB to USB-C Wire
USB to USB-C wire to have our Raspberry Pi control our U2D2 Controller.

##### More wires:
Many of the wires to connect our battery holders to the DC/DC converter, and our DC/DC converter to give power to the Raspberry Pi, U2D2 controller, and Pi Pico

#### LEGOs: 
We used LEGOs as our base and to structure our robot. 

### Reason for Chosen Components:

(include testing or iterations affecting performance)

## Power & Sensor Architecture:

**Power budget:**
 
**Sensor trade-offs:** Include placement justified using field geometry; calibration method; failure point considerations; iteration evidence

**Diagrams:**

**Wiring** 

**Current strategy:**

**Sensor choices/placement:**

**Calibration:**

**More Diagrams:**

## Software Architecture & Obstacle Strategy:
Stuff

### Code structure:
State machine with rationale; algorithm justification (PID, CV, IMU, etc.); handling edge cases; testing/tuning process; metrics used to validate performance

**Modules:**

**Lane Following:** 

**Obstacle Logic:**

**Algorithm Explanation:**

## Systems Thinking & Engineering Decisions:

**Subsystem interactions**

**Constraints, Trade-offs, and Risk Analysis:***

**Iteration cycles:**

**Engineering Reasoning:**

## Conclusion: 

