![Arduino](https://img.shields.io/badge/Arduino-Uno-blue?logo=arduino)
![PID](https://img.shields.io/badge/Control-PID-green)
![SolidWorks](https://img.shields.io/badge/3D%20Design-SolidWorks-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)# Arduino-Project---Dual-PID-Control-system

This project implements **two interacting PID controllers** in a robotic system: one for maintaining balance and another for controlling distance from a wall. The combination of these two control loops allows the robot to **actively stabilize itself while following a set distance**, making it a unique integration of dual PID systems in one platform.

---

## Overview
This project required a strong understanding of **PID control systems** and the **mathematical reasoning** behind each tuning parameter (Proportional, Integral, Derivative).  

The uniqueness of this design lies in how the **Balancing PID** and **Distance PID** interact with each other. I created variables for the outputs of an **MPU6050 gyroscope** and an **ultrasonic sensor**, then combined them into a single `CombinedOutput` value. This value drives the **DC motors**, enabling the robot to **maintain its balance** while also **tracking a set distance from a wall**.

Additionally, I used **SolidWorks** to design and 3D-print the frame for mounting all components. I also gained hands-on experience with **soldering components to PCB boards**, improving the stability of the gyroscope and ensuring **accurate accelerometer and gyroscope readings**.

---

## How It Works
1. **Balance Control:**  
   - Reads tilt data from the MPU6050 (gyroscope + accelerometer).  
   - A PID controller adjusts motor speed to maintain an upright position.  

2. **Distance Control:**  
   - Reads distance data from an ultrasonic sensor (HC-SR04).  
   - A second PID controller adjusts movement to maintain a target distance from a wall.  

3. **Combined Output:**  
   - The two PID outputs are merged into one `CombinedOutput` value.  
   - This value drives the DC motors, allowing simultaneous balance and distance tracking.

---

## Hardware Used
- **Arduino Uno**
- **MPU6050** (Gyroscope & Accelerometer)
- **HC-SR04** Ultrasonic Sensor
- **DC motors** with **L298N motor driver**
- **Custom 3D-printed frame** (SolidWorks)

---

## Setup & Installation
1. **Clone this repository:**
   ```bash
   git clone https://github.com/YourUsername/Arduino-Project---Dual-PID-Control-system.git
