# Ball-Balance-Table
Ball Balance Table Project
This project is a Ball Balance Table that uses computer vision and motor control to stabilize and navigate a ball on a tilting platform. It combines AVR microcontrollers for motor control and a Raspberry Pi for image processing, ball tracking, and maze solving.

🧠 Project Overview
A Raspberry Pi captures real-time video to detect the ball’s position.

It uses computer vision to track the ball and solve mazes if present.

The Raspberry Pi sends control signals to AVR microcontrollers, each responsible for a DC motor.

DC motors tilt the platform in two axes, simulating servo-like behavior to balance or navigate the ball.

🧰 Technologies Used
🖥️ Raspberry Pi (High-Level Control)
Python + OpenCV for ball detection and tracking

Maze-solving algorithms

UART communication with AVR microcontrollers

🔧 AVR Microcontrollers (Low-Level Motor Control)
C for firmware

PID control logic to act like servos

Direct control of DC motors

⚙️ Hardware
2 DC motors (X and Y axes)

Camera (USB or PiCamera)

Raspberry Pi (Model 3/4 recommended)

AVR microcontrollers (e.g., ATmega32 or similar)

Tilt platform with ball

🎥 How It Works
The camera captures frames and detects the ball’s position.

The Raspberry Pi calculates the required tilt using a control algorithm.

It sends appropriate movement commands over UART to each AVR board.

The AVRs run a PID loop to smoothly control the DC motors to the target angles.

In maze-solving mode, the Pi plans a path and guides the ball step-by-step.

🚀 Getting Started
Raspberry Pi
Set up Python and OpenCV

Run the ball tracking and maze-solving script

Connect to AVRs via UART (TX/RX)

AVR Microcontrollers
Flash the C firmware that receives position commands and drives the motors

Tune PID parameters for stability
