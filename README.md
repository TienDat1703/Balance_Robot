# Balance_Robot
*Course:* Embedded Systems Design  (CE224.Q12)

*Institution:* University of Information Technology (UIT) - VNU-HCM

*Instructor:* M.S. Tran Ngoc Duc

## 📖 About The Project (Overview)

The **Mini Self-Balancing Robot** is a foundational benchmark model in automatic control theory, designed around the classical **Inverted Pendulum** problem. It addresses complex challenges in real-time sensor processing, data fusion, and advanced control loops. 

Developing this miniature two-wheeled robotic vehicle using the high-performance **STM32F4** microcontroller allows for optimized computational performance, precise execution constraints, and enhanced physical stability during live testing.

### 🎯 Project Objectives
To achieve full stabilization and counter external forces, the project focuses on three primary goals:
1. **Hardware & Mechanical Design:** Successfully design and assemble a robust, functional two-wheeled vehicle frame capable of maintaining an upright position independently.
2. **Precision Sensor Fusion:** Integrate and deploy a **Kalman Filter** algorithm to eliminate measurement noise caused by high-frequency motor vibrations, capturing highly accurate tilting angles.
3. **Advanced Cascade Motion Control:** Implement a **Cascade (Dual-Loop) PID Controller** to simultaneously regulate the orientation (balance) and physical velocity of the robot, ensuring it stands firmly without drifting away from its baseline coordinates.

## 🛠 Hardware Architecture & Components

The mechanical structure and electronics are selected to optimize physical response times and processing efficiency:

| Component | Specification | Description |
| :--- | :--- | :--- |
| **Microcontroller** | STM32F407VET6 BlackBoard | ARM Cortex-M4 @ 168 MHz, 512KB Flash, 192KB RAM, FPU enabled |
| **IMU Sensor** | MPU-6050 | 3-axis accelerometer & 3-axis gyroscope connected via $I^2C$ |
| **Actuators** | 2 x JGA25-370 DC Servo | 12VDC, 130RPM, 46.8:1 reduction ratio with dual-channel AB quadrature encoders |
| **Motor Driver** | L298N H-Bridge Module | Dual-channel DC driver supporting up to 2A continuous per motor |
| **Power Supply** | 3 x 18650 Li-ion Batteries | 11.1V nominal package |
| **Voltage Regulator**| LM2596 DC-DC Buck | Steps down battery voltage to stable 5V logic power |
| **Chassis** | Acrylic Plate | $8 \times 20$ cm frame with 65mm wheels |
| **Programmer** | ST-LINK V2 | SWD protocol interface for firmware flashing and real-time debugging |

## ⚙️ Peripherals & STM32 Pin Configuration

The firmware is developed using **STM32CubeIDE** utilizing the **HAL (Hardware Abstraction Layer)**. Peripherals are tailored as follows:

* **System Core:**
    * **RCC:** High-Speed External (HSE) clock configured with an 8MHz external crystal resonator.
    * **SYS:** Serial Wire Debug (SWD) configuration for ST-LINK communication.
* **Connectivity ($I^2C$):**
    * **I2C1 (PB8 - SCL, PB9 - SDA):** Standard mode at 100 kHz for polling MPU-6050 registers.
* **Timers & PWM Generation:**
    * **TIM3 (PA6 - CH1, PA7 - CH2):** Configured for high-frequency PWM generation (Prescaler: 3, ARR: 999) to drive motor channels smoothly.
    * **TIM2 (PA0/PA1) & TIM4 (PD12/PD13):** Hardware Encoder Mode configured to capture dual-channel quadrature encoder feedback from the left/right wheels.
    * **TIM5 (Interrupt Mode):** Core execution tick running at exactly **200Hz** ($84\text{MHz} / (8399+1) / (49+1)$) creating an absolute 5ms deterministic control window.
* **GPIO Output (Direction Controls):**
    * **PB0, PB1:** Left motor directional configuration.
    * **PE7, PE8:** Right motor directional configuration.

## 🧠 Algorithmic Implementation

### 1. Sensor Fusion: Kalman Filter
Raw data from a gyroscope suffers from low-frequency drift over time, whereas accelerometer readings are heavily corrupted by high-frequency vibration noises from the motors. The Kalman filter handles this by combining the predictive gyro-rate vector with structural accelerometer angles:
* **Predict State:** Establishes pitch prediction using angular velocity and updates the error covariance matrix.
* **Measurement Update:** Compares the predicted projection with the static gravity vector to calculate the optimal Kalman gain, producing a clean, chatter-free `robot_angle`.

### 2. Control Strategy: Cascade PID
To prevent the robot from drifting continuously due to geometric tolerances or uneven terrains, a **Dual-Loop (Cascade) PID** controller is deployed:
* **Inner Loop (Balance Loop):** Executes every 5ms to keep the robot vertical.
    $$\mathit{balance\_pwm} = K_{\mathit{p\_bal}} \times \mathit{angle\_error} + K_{\mathit{d\_bal}} \times (0 - \mathit{gyro\_y\_rate}) + K_{\mathit{i\_bal}} \times \mathit{angle\_integral}$$
* **Outer Loop (Velocity Loop):** Polls encoder data to evaluate the linear shift. If a drift trend is captured, it updates the `target_angle` parameter of the inner loop, forcing the physical center of mass to pitch slightly backward/forward to correct position tracking.

### 3. Non-Linear Deadzone Offset
Due to static friction in small metal gearboxes, the motors fail to turn if the raw calculated PWM is too small. A hard-coded threshold `PWM_DEADZONE = 300.0f` is added to any active non-zero output, bypassing sluggish micro-oscillations and boosting system responsiveness.

## 📂 Repository Structure
```text
balance_robot/
├── Core/
│   ├── Inc/
│   │   ├── mpu6050.h          # MPU-6050 registers configuration & function headers [cite: 2, 45]
│   │   └── main.h             # HAL definitions & pin naming conventions
│   └── Src/
│       ├── mpu6050.c          # I2C driver routines & sensor processing logic [cite: 2, 49]
│       ├── stm32f4xx_it.c     # Interrupt service routines (TIM5 ticks handler)
│       └── main.c             # Core system init, Kalman algorithm, Cascade PID loop [cite: 2, 54, 57]
├── Drivers/                   # STMicroelectronics HAL standard driver libraries [cite: 11]
├── .mxproject                 # STM32CubeMX structural reference mapping
└── balance_robot.ioc          # Hardware layout graphic configuration template
```
---

## ⚙️ **Installation & Setup**
Prerequisites

  1. STM32CubeIDE (v1.13.0 or later recommended).  

  2. An ST-LINK V2 hardware flasher.  

Flashing Code to Hardware
  1. Clone this repository onto your machine:
       git clone https://github.com/TienDat1703/Balance_Robot
  2. Launch STM32CubeIDE and open the project directory via File > Open Projects from File System...
  3. Connect the ST-LINK V2 to your laptop and link it to the SWD pins on your STM32 Blackboard (SWDIO, SWCLK, GND, 3.3V).
  4. Click Build (the hammer icon) to verify there are no compilation errors.
  5. Click Run or Debug (the bug icon) to flash the software directly onto your microcontroller memory.

## 👥 **Contributors & Roles**

| Student Name | Student ID | Core Project Role & Key Contributions |
| :--- | :--- | :--- |
| **Ngo Tien Dat** | 23520254 | **Firmware Engineer**<br>• Configured ultrasonic motor PWM frequency ($21.0\text{ kHz}$).<br>• Implemented non-linear PWM deadzone compensation ($\pm 300.0\text{ f}$).<br>• Handled directional H-bridge logic and PWM output scaling. |
| **Tran Quang Nhat** | 23521102 | **Hardware & Encoder Engineer**<br>• Managed hardware architecture design and circuitry layout.<br>• Handled quadrature encoder signal capturing and register decoding. |
| **Le Hung Phat** | 23521139 | **Algorithm Engineer**<br>• Formulated mathematical analysis modeling for the Inverted Pendulum.<br>• Developed and integrated the multi-sensor Kalman Filter matrix processing. |
| **Nguyen Thanh Hieu** | 23520486 | **Mechanical & Control Analyst**<br>• Drafted the mechanical chassis structure and physical body layout.<br>• Handled real-time empirical PID parameter tuning and drift monitoring. |
