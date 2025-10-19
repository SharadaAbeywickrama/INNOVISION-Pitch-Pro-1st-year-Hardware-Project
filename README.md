# Pitch Pro – Autonomous Outdoor Line Painting Robot

## Description

Pitch Pro is a microcontroller-based autonomous robot designed to paint precise straight and curved lines on outdoor surfaces such as sports fields and construction sites. Developed by first-year undergraduates, this system uses Arduino Mega and ESP32 boards for distributed task management. The robot focuses on achieving maximum precision for rectangle and square line painting using PID control for straight lines and 90-degree turn correction with a 2-attempt loop system.

The robot is capable of drawing predefined shapes (straight lines, circles, rectangles, squares) and features GPS tracking for path visualization on the web interface. It integrates sensor feedback, paint level monitoring, and obstacle detection to ensure reliable operation on outdoor terrain.

---

## Features

- **Autonomous outdoor line painting** with adjustable spray width
- **Shape-based path painting** (straight, circle, rectangle, square)
- **Precise 90° turning** with correction algorithm (2-attempt loop)
- **PID-controlled movement** for straight line accuracy
- **GPS path tracking** visualization on web interface (Neo-6M)
- **Paint level monitoring** with ultrasonic sensor and LED indicators
- **Obstacle detection and avoidance** using dual front ultrasonic sensors
- **Adjustable line width** with servo-controlled rack and pinion actuators
- **Paint mixing mechanism** with dedicated motor system
- **Multi-interface control**:
  - Web-based interface for remote operation
  - TFT touchscreen for direct control
  - 4×3 keypad for outdoor ease of use
- **Status indication system** with LEDs and buzzer alerts
- **Modular design** for future upgrades (RTK GPS, reference points)

---

## Hardware Components

### Controllers & Interface
- **Arduino Mega** – Main controller (motors, sensors, paint system, obstacle detection)
- **ESP32** – Wi-Fi communication, GPS, TFT display, web interface
- **TFT 2.4″ SPI Display** – User interface and status display
- **4×3 Matrix Keypad** – Manual control for outdoor operations
- **GPS Neo-6M Module** – Path tracking (connected to ESP32)

### Drive System
- **Wiper Motors (x2)** – Main drive motors with 6-inch wheels (center-mounted)
- **BTS7960 Motor Drivers (x2)** – High-current motor control for wiper motors
- **Caster Wheels (x4)** – Support wheels for stability
- **IRF540N MOSFETs** – LED and auxiliary switching
- **No wheel encoders** – Relies on PID control and turn correction

### Paint System
- **12V Diaphragm Pump** – Paint spraying mechanism
- **5V Relay Module** – Diaphragm pump control
- **12V Motor with L298N Driver** – Paint mixing mechanism
- **MG90S Servo Motors (x2)** – Line width adjustment via rack and pinion
- **Ultrasonic Sensor (HC-SR04)** – Paint level measurement

### Sensors & Monitoring
- **Ultrasonic Sensors HC-SR04 (x2)** – Front obstacle detection
- **Ultrasonic Sensor HC-SR04 (x1)** – Paint level monitoring
- **MPU6050/6500** – Gyroscope/accelerometer for PID control
- **QMC5883L Magnetometer** – Heading verification for turns

### Status & Alerts
- **12V T10 LEDs (x4)** with IRF540N MOSFETs:
  - 3 LEDs for paint level indication (High/Medium/Low)
  - 1 LED for robot shadow/status indication
- **5V Buzzer** – Status alarms and alerts

### Power & Control
- **Primary Power**: 5200mAh LiPo Battery – Powers wiper motors and diaphragm pump
- **Secondary Power**: 3200mAh 11.1V Li-ion Battery Pack – Powers mixing motor and electronics
- **LM2596 Buck Converter** – 12V to 5V step-down conversion
- **5V Relay Module** – Pump control
- **IRF540N MOSFETs** – LED and auxiliary switching
- **Switch, fuse, and power distribution**

---

## Software Architecture

### Arduino Mega Responsibilities
- PID control for straight line movement
- BTS7960 motor driver control for wiper motors
- 90-degree turn correction (2-attempt loop)
- Paint mixing motor control (L298N driver)
- Sensor reading (ultrasonic, gyroscope, magnetometer)
- Paint system control (pump relay, servo actuators)
- Status LED control and buzzer alerts
- Keypad input processing
- Obstacle detection and avoidance
- Power management coordination

### ESP32 Responsibilities
- Wi-Fi server and web interface
- TFT display management
- GPS data collection (Neo-6M)
- Path tracking and web visualization
- Communication with Arduino Mega
- Real-time status updates

---

## File Structure

```
/PitchPro
├── Microcontroller_Codes/
│   ├── mega_code.ino/            
│   ├── esp32_code.ino    
│  
├── Web_Interface/
│  
├── Development/
│   ├── motor_tests/             # Drive motor testing
│   ├── sensor_calibration/      # Sensor setup scripts
│   ├── PID testing straight/    # Adjust PID values
│   ├── Turn90/                  # Optimizes turn 90
│   ├── obstacle detection/      # Obstacle Stop
│   ├── paint_system_test/       # Paint mechanism testing
│   └── gps_test/                # GPS functionality testing
├── Media/
│   ├── photos/                  # Project photos
│   └── videos/                  # Demo and testing videos
├── Documents/
│   └── PitchPro_Report.pdf      # Final project report
└── README.md                    # This documentation
```

---

## System Operation

### 1. User Input
- **Web Interface**: Shape selection and parameters
- **TFT Screen**: Direct touch control and status
- **Keypad**: Manual outdoor operation (4×3 matrix)

### 2. Navigation & Control
- **Straight Lines**: PID control using gyroscope feedback
- **90° Turns**: Turn correction algorithm with 2-attempt loop for precision
- **No Encoders**: Relies on time-based movement and sensor feedback

### 3. Paint System
- **Level Monitoring**: Ultrasonic sensor measures paint level
- **Visual Indicators**: 3 LEDs show High/Medium/Low paint levels
- **Mixing**: Dedicated motor keeps paint consistency
- **Width Adjustment**: Dual servos with rack and pinion for line width
- **Pump Control**: 5V relay controls 12V diaphragm pump

### 4. Safety & Monitoring
- **Obstacle Detection**: Dual front ultrasonic sensors
- **Status LED**: Shows robot operational state
- **Audio Alerts**: 5V buzzer for various status conditions
- **GPS Tracking**: Path visualization (accuracy limited by Neo-6M)

---

## Precision & Limitations

### Current Capabilities
- **Successful rectangle/square painting** with PID-controlled straight lines
- **90-degree turn correction** using 2-attempt loop system
- **Paint level monitoring** and automatic alerts
- **Basic GPS tracking** for path visualization
- **Obstacle avoidance** for safe operation

### Known Limitations
- **GPS Accuracy**: Neo-6M provides limited precision for navigation
- **No RTK GPS**: Advanced positioning not implemented
- **No Reference Points**: Complex waypoint navigation not available
- **Weather Dependent**: Outdoor performance varies with conditions

### Future Development Potential
- **RTK GPS Integration** for centimeter-level accuracy
- **Reference Point Navigation** for complex field mapping
- **Advanced Path Planning** with waypoint systems
- **Weather Compensation** algorithms
- **Remote Fleet Management** capabilities

*As first-year undergraduates, this project represents our current technical capabilities while establishing a foundation for future enhancements through continued university research and learning.*

---

## Media

📸 **Photos** – [Project Photos Link Here]

🎥 **Videos** – [Demo Videos Link Here]

---

## Technical Documentation

📄 **[PitchPro_Report.pdf](Documents/PitchPro_Report.pdf)**

---

## Software Requirements

### Development Environment
- **Arduino IDE** – For Mega and ESP32 programming
- **ESP32 Board Package** – Arduino IDE ESP32 support

### Required Libraries
```
// Arduino Mega Libraries
#include <Wire.h>              // I2C communication
#include <SPI.h>               // SPI communication
#include <Servo.h>             // Servo motor control
#include <Keypad.h>            // 4x3 keypad interface

// ESP32 Libraries
#include <WiFi.h>              // Wi-Fi connectivity
#include <WebServer.h>         // HTTP server
#include <TFT_eSPI.h>          // TFT display
#include <SoftwareSerial.h>    // GPS communication

// Sensor Libraries
#include <MPU6050.h>           // Gyroscope/accelerometer
#include <QMC5883L.h>          // Magnetometer
#include <NewPing.h>           // Ultrasonic sensors (optional)
```

---

## Installation & Setup

### 1. Hardware Assembly
- Mount wiper motors with BTS7960 drivers and 6-inch wheels (center position)
- Install 4 caster wheels for support
- Connect dual battery system (5200mAh LiPo + 3200mAh Li-ion)
- Install LM2596 buck converter for voltage regulation
- Connect paint system (pump, mixing motor with L298N, servos)
- Wire sensor arrays (ultrasonic, gyroscope, GPS)
- Install status LEDs with IRF540N MOSFETs and buzzer
- Connect keypad and TFT display

### 2. Software Installation
```bash
# Clone the repository
git clone https://github.com/yourusername/PitchPro.git
cd PitchPro

# Install required libraries in Arduino IDE
# Upload mega_code.ino/ to Arduino Mega
# Upload esp32_code.ino/ to ESP32 board
```

### 3. Calibration
- **Gyroscope**: Calibrate for straight-line accuracy
- **Paint System**: Test pump pressure and servo positions
- **Turn Correction**: Adjust 90-degree turn parameters
- **GPS**: Verify satellite connection (outdoor testing)

### 4. Operation
- **Power up** dual battery system (5200mAh + 3200mAh)
- **Verify** LM2596 converter provides stable 5V output
- **Connect** to Wi-Fi network or use keypad
- **Check paint level** and fill if necessary
- **Test BTS7960** motor drivers and wiper motor response
- **Select shape** and start painting operation

---

## Project Status

✅ **Successfully achieves** precise rectangle and square painting  
✅ **PID control** provides straight line accuracy  
✅ **Turn correction algorithm** ensures 90-degree precision  
✅ **Paint level monitoring** with visual and audio alerts  
✅ **Multi-interface control** (web, TFT, keypad)  
✅ **GPS path tracking** for visualization  
✅ **Obstacle detection** and avoidance  

🔄 **Future enhancements** planned with advanced university coursework

---

## Contributors

*First-year undergraduate engineering students*  
[Add team member names here]

---

## License

[Add your license here]

---

## Acknowledgments

Special thanks to our university supervisors and the engineering department for supporting this autonomous robotics project. This system demonstrates the practical application of control systems, embedded programming, and mechanical design principles learned in our first year of study.
