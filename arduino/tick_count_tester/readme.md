# Encoder Simulation with Arduino and ROS

This project provides a simple way to simulate wheel encoders using push buttons connected to an Arduino. The tick counts are published over ROS using `rosserial`, and a ROS service is included to reset both encoders to a value near the maximum (to demonstrate rollover behavior).

---

## ✨ What This Code Does

- Counts encoder ticks using interrupts on pins 2 and 3.
- Uses buttons to simulate forward/reverse encoder signals.
- Publishes tick counts to two ROS topics:
  - `/left_ticks`  (std_msgs/Int16)
  - `/right_ticks` (std_msgs/Int16)
- Offers a ROS service `/set_to_near_max` to set both encoder counts to 32760.
- Demonstrates 16-bit rollover: 32767 ➔ -32768.

---

## ⚙ Hardware Setup

### Required:
- Arduino (Uno, Mega, or compatible)
- 4 Push Buttons
- 4 10kΩ pull-down resistors *(optional if using INPUT_PULLUP)*
- Breadboard and jumper wires

### Pin Mapping:
| Purpose           | Arduino Pin |
|------------------|-------------|
| Left Encoder A   | 2           |
| Right Encoder A  | 3           |
| Left Encoder B   | 4           |
| Right Encoder B  | 5           |

### Wiring Each Button:
Each encoder uses two signals: A (interrupt pin) and B (direction). For simulation:
- Connect one side of each button to GND.
- Connect the other side to the appropriate Arduino pin.
- The sketch uses `INPUT_PULLUP`, so no resistor is needed.

Button press = signal goes LOW = interrupt triggered.

---

## ⚙ Arduino Sketch Requirements

### Install Required Arduino Libraries:
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)
- [rosserial](http://wiki.ros.org/rosserial)

Use the Arduino IDE Library Manager or manually place the libraries in your `libraries/` folder.

---

## ⚡ ROS Setup (PC Side)

### 1. Upload the Sketch
Use the Arduino IDE to upload this sketch to your Arduino.

### 2. Connect Arduino to ROS:
```bash
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```
Adjust `/dev/ttyUSB0` as needed (e.g. `/dev/ttyACM0`).

### 3. Watch the Encoder Ticks:
```bash
rostopic echo /left_ticks
rostopic echo /right_ticks
```

### 4. Trigger Rollover Test:
```bash
rosservice call /set_to_near_max
```
Then tap the buttons a few times to roll past 32767.

---

## 🎓 Educational Use
- Demonstrates how quadrature encoders work.
- Simulates direction detection via dual-channel logic.
- Explains rollover behavior in 16-bit integers.
- Encourages experimentation with encoder math, overflow handling, and services.

---

## 🏆 Extra Credit Ideas
- Add `/reset_ticks` service to set both values to 0.
- Add `/set_ticks` custom service to set a specific value.
- Add logic to calculate velocity (ticks/sec).
- Display tick values on an LCD.

---
