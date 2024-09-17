# AMR Control System

A comprehensive Autonomous Mobile Robot (AMR) control system featuring dual-motor control with encoder feedback, PID control, and wireless communication capabilities.

## Project Overview

This project implements a complete motor control system for an autonomous mobile robot using AVR microcontrollers and ESP32 for wireless communication. The system supports dual-motor control with real-time encoder feedback, PID-based velocity control, and serial communication for command input and telemetry output.

## Features

- Dual motor PWM control with bidirectional operation
- Dual encoder support (analog-based magnetic encoders)
- PID control for precise velocity regulation
- UART serial communication for command interface
- ESP32-based wireless serial bridge for remote control
- Real-time encoder position tracking and turn counting
- Configurable PID parameters for both motors

## Hardware Requirements

### AVR Motor Controller
- **Microcontroller**: ATmega328P (Arduino Uno or compatible)
- **Clock Frequency**: 16 MHz
- **Motors**: 2x DC motors with H-bridge driver
- **Encoders**: 2x analog magnetic encoders (connected to A0, A1)
- **Communication**: UART (9600 baud)

### ESP32 Serial Bridge
- **Microcontroller**: ESP32
- **Communication**: WiFi, UART (57600 baud)
- **Purpose**: Wireless serial data relay

## Pin Configuration

### AVR Motor Controller

#### Motor Control Pins
- Motor 1 PWM: PD6 (Digital Pin 6, OC0A)
- Motor 1 Direction: PD2 (Digital Pin 2)
- Motor 2 PWM: PD5 (Digital Pin 5, OC0B)
- Motor 2 Direction: PD3 (Digital Pin 3)

#### Encoder Pins
- Encoder 1: A1 (Analog Input 1)
- Encoder 2: A0 (Analog Input 0)

#### Communication
- TX: Digital Pin 1
- RX: Digital Pin 0

## Project Structure

```
avr_codes/
├── avr_motor_controller/
│   └── final.ino              # Main AVR motor control firmware
├── esp32_serial_bridge/
│   └── serial_to_http.ino     # ESP32 wireless serial bridge
├── libraries/
│   ├── ArduinoJson/           # JSON parsing library
│   ├── AS5600/                # Magnetic encoder library
│   ├── AsyncTCP/              # Async TCP for ESP32
│   ├── ESPAsyncTCP/           # ESP Async TCP library
│   └── ESPAsyncWebServer/     # ESP32 web server library
├── .gitignore
└── README.md
```

## Software Architecture

### AVR Motor Controller

The AVR firmware implements a complete motor control system with the following components:

#### Core Modules
1. **Motor Control**: PWM generation using Timer0 for dual-motor speed and direction control
2. **Encoder Reading**: ADC-based analog encoder reading with turn counting
3. **PID Controller**: Dual PID loops for independent motor velocity control
4. **Serial Communication**: UART-based command and telemetry interface
5. **Timer Management**: Timer2 for 1ms interrupt-driven encoder updates and PID execution

#### Control Loop
- Encoder update rate: 1 kHz (1ms interval via Timer2)
- PID update rate: 3 Hz (333ms interval)
- PID calculations run in interrupt context for precise timing

### ESP32 Serial Bridge

The ESP32 acts as a wireless bridge, forwarding serial data from the AVR controller to HTTP clients over WiFi.

## Communication Protocol

### Command Format

Commands are sent via UART as ASCII strings terminated by newline (`\n`) or carriage return (`\r`).

#### Motor Speed Command (Open Loop)
```
<speed1> <speed2>\n
```
- `speed1`: Motor 1 speed (-255 to 255)
- `speed2`: Motor 2 speed (-255 to 255)
- Negative values indicate reverse direction

Example: `100 -50\n` (Motor 1 forward at 100, Motor 2 reverse at 50)

#### PID Target Command
```
m<count1> <count2>\n
```
- `count1`: Target encoder ticks per frame for Motor 1
- `count2`: Target encoder ticks per frame for Motor 2

Example: `m10 10\n` (Set both motors to 10 ticks/frame)

#### Encoder Query
```
e\n
```
Response: `<encoder1_count> <encoder2_count>\n`

#### Reset Command
```
r\n
```
Stops motors, resets PID, and clears encoder counts.

## PID Configuration

### Motor 1 (Left Motor)
- Proportional gain (Kp): 13
- Derivative gain (Kd): 7
- Integral gain (Ki): 0
- Output scaling (Ko): 1

### Motor 2 (Right Motor)
- Proportional gain (Kp): 20
- Derivative gain (Kd): 9
- Integral gain (Ki): 0
- Output scaling (Ko): 1

These parameters can be tuned in the source code based on motor characteristics and desired response.

## Installation

### AVR Motor Controller

1. Open `avr_motor_controller/final.ino` in Arduino IDE
2. Select board: Arduino Uno (or ATmega328P)
3. Select appropriate COM port
4. Upload the sketch

### ESP32 Serial Bridge

1. Install required libraries:
   - AsyncTCP
   - ESPAsyncWebServer
   
2. Update WiFi credentials in `esp32_serial_bridge/serial_to_http.ino`:
   ```cpp
   const char* ssid = "YOUR_SSID";
   const char* password = "YOUR_PASSWORD";
   ```

3. Open the sketch in Arduino IDE
4. Select board: ESP32 Dev Module
5. Upload the sketch

6. Access serial data via HTTP:
   ```
   http://<ESP32_IP>/
   ```

## Usage

### Basic Operation

1. Power up the AVR controller and motors
2. Connect via serial terminal (9600 baud)
3. Send commands:
   - Set motor speeds: `150 150` (both motors forward)
   - Enable PID control: `m20 20` (target velocity)
   - Query encoders: `e`
   - Stop motors: `0 0`

### With ESP32 Bridge

1. Ensure ESP32 is connected to AVR via serial
2. Connect ESP32 to WiFi network
3. Access the ESP32 IP address in a web browser to view serial data
4. Send commands through the serial interface

## Technical Details

### Encoder Resolution
- Analog encoders provide 0-360 degree feedback
- Turn counting via quadrant tracking (4 quadrants per revolution)
- Resolution: ~1024 ADC steps per revolution

### PWM Specifications
- Frequency: ~976 Hz (16 MHz / (64 prescaler * 256))
- Resolution: 8-bit (0-255)
- Mode: Fast PWM, non-inverted

### Timer Configuration
- Timer0: Fast PWM for motor control
- Timer2: CTC mode, 1ms interrupt for encoder/PID updates
- Prescaler: 64

## Troubleshooting

### Motors not responding
- Check motor driver connections
- Verify power supply to motors
- Check PWM pin connections
- Test with direct speed commands (not PID mode)

### Erratic encoder readings
- Check analog encoder connections
- Verify encoder power supply (5V)
- Ensure proper magnetic alignment
- Check ADC reference voltage configuration

### PID instability
- Reduce Kp gain if oscillating
- Increase Kd gain for damping
- Verify encoder feedback is working correctly
- Check for mechanical binding

### ESP32 not connecting
- Verify WiFi credentials
- Check serial baud rate (57600)
- Ensure proper TX/RX connections
- Monitor serial output for connection status
