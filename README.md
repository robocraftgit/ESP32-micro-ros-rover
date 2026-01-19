# ESP32 ROS 2 Rover using micro-ROS

##  Project Overview

This project implements a **Wi-Fi–controlled ESP32-based differential drive rover** using **ROS 2** and **micro-ROS**.

The rover is teleoperated using the `teleop_twist_keyboard` package, where velocity commands (`cmd_vel`) are sent from a ROS 2 system to an ESP32 over Wi‑Fi using the **micro-ROS agent**. The ESP32 subscribes to the `cmd_vel` topic and directly controls the motor driver GPIO pins.

---

##  Key Features

* ESP32-based differential drive rover
* ROS 2 (Humble) integration via micro-ROS
* Wireless control over Wi‑Fi (no USB needed during runtime)
* Teleoperation using `teleop_twist_keyboard`
* Simple GPIO-based motor control (no PWM required)

---

##  Hardware Requirements

* ESP32 development board
* Dual DC motor driver (L298N / L293D / TB6612FNG)
* 4 × DC geared motors
* Robot chassis 
* External motor power supply (battery)
* Jumper wires

---

##  Motor Pin Configuration (ESP32)

| Motor       | Pin | Function |
| ----------- | --- | -------- |
| Left Motor  | IN1 | GPIO 26  |
| Left Motor  | IN2 | GPIO 27  |
| Right Motor | IN3 | GPIO 33  |
| Right Motor | IN4 | GPIO 13  |

>  Enable pins on the motor driver are **not used**. Motors are driven using direction control only.

---

## 💻 Software Requirements

### On PC (ROS 2 side)

* Ubuntu 22.04
* ROS 2 Humble
* micro-ROS agent
* `teleop_twist_keyboard`

### On ESP32

* Arduino IDE
* ESP32 board support
* micro_ros_arduino library

---

##  Network Setup

* ESP32 and PC must be connected to the **same Wi‑Fi network**
* Note the **PC IP address** and use it in the ESP32 code as `AGENT_IP`

---

##  How to Run

###  Start micro-ROS Agent (PC)

```bash
micro-ros-agent udp4 --port 8888
```

###  Upload Code to ESP32

* Connect ESP32 via USB
* Select correct board and port in Arduino IDE
* Upload the firmware

###  Power the Rover

* Disconnect USB (optional)
* Power ESP32 and motor driver using external supply

###  Start Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keyboard controls to move the rover.

---

## ROS Topics

| Topic      | Type                  | Direction           |
| ---------- | --------------------- | ------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribed by ESP32 |

---

##  Debugging Tips

* Use `Serial Monitor` at **115200 baud** for ESP32 logs
* Check Wi‑Fi connection and IP address
* Ensure micro-ROS agent is running **before** powering ESP32
* Verify topic using:

```bash
ros2 topic echo /cmd_vel
```

---

##  Future Improvements

* Add wheel encoders for odometry
* Publish `/odom` and `/tf`
* Integrate IMU (MPU6050 / BNO055)
* Add ultrasonic or LiDAR sensors
* Implement autonomous navigation (Nav2)
* Add PWM speed control

---

##  Learning Outcomes

* micro-ROS fundamentals
* ROS 2 communication (`cmd_vel`)
* ESP32 Wi‑Fi transport
* Differential drive robot control
