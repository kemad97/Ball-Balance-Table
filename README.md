# Ball-Balance-Table

A smart ball-balancing system that uses **computer vision** and **motor control** to keep a ball stable—or even solve a maze—on a tilting platform. The system combines **AVR microcontrollers** for real-time motor control and a **Raspberry Pi** for image processing, ball tracking, and pathfinding.

---

## 🧠 Overview

* 📷 A **Raspberry Pi** captures real-time video to detect the ball’s position using **OpenCV**.
* 🧭 It processes the position and, if in maze mode, solves a path to the goal.
* 🔁 Control signals are sent via **UART** to **AVR microcontrollers**, each driving a DC motor.
* ⚙️ The motors tilt the platform in two axes to move or stabilize the ball using a **PID controller**.

---

## 🧰 Technologies Used

### 🖥️ Raspberry Pi (High-Level Control)

* Python & OpenCV for computer vision
* Maze-solving algorithms
* UART serial communication with AVRs

### 🔧 AVR Microcontrollers (Low-Level Motor Control)

* C for firmware development
* PID control logic to simulate servo-like motor behavior
* Direct control of DC motors via H-bridges

### ⚙️ Hardware Components

* 2 DC motors (for X and Y axes)
* USB or PiCamera module
* Raspberry Pi (Model 3 or 4 recommended)
* AVR microcontrollers (e.g., ATmega32)
* Tiltable platform + ball

---

## 🎥 How It Works

1. **Camera** captures video and detects the ball’s coordinates.
2. **Raspberry Pi** calculates the required tilt using a control or pathfinding algorithm.
3. **AVR microcontrollers** receive commands over UART and adjust the DC motors using PID control.
4. In **maze-solving mode**, the Pi plans a path and guides the ball cell by cell toward the goal.

---

## 🚀 Getting Started

### 🔌 Raspberry Pi Setup

1. Install Python, OpenCV, and required dependencies.
2. Clone the project and run the `tracking.py` or `maze_solver.py` scripts.
3. Connect to the AVR boards via UART (TX/RX).

### ⚙️ AVR Microcontroller Setup

1. Flash the AVR firmware (`motor_control.c`) using your preferred ISP programmer.
2. Ensure each AVR handles one axis (X or Y).
3. Tune PID parameters in the firmware for optimal performance.

---

## 📷 Demo

[![Watch the Demo](https://img.youtube.com/vi/VCe8QrZcQyY/0.jpg)](https://youtu.be/VCe8QrZcQyY)


---

## 👤 Author

**Kerolos Emad Eid**
Passionate about embedded systems, computer vision, and robotics.

