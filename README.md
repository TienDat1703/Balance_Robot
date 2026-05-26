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

[cite_start]The mechanical structure and electronics are selected to optimize physical response times and processing efficiency[cite: 4]:

| Component | Specification | Description |
| :--- | :--- | :--- |
| **Microcontroller** | [cite_start]STM32F407VET6 BlackBoard [cite: 12] | [cite_start]ARM Cortex-M4 @ 168 MHz, 512KB Flash, 192KB RAM, FPU enabled [cite: 13, 15] |
| **IMU Sensor** | [cite_start]MPU-6050 [cite: 21] | [cite_start]3-axis accelerometer & 3-axis gyroscope connected via $I^2C$ [cite: 21, 28] |
| **Actuators** | [cite_start]2 x JGA25-370 DC Servo [cite: 22] | [cite_start]12VDC, 130RPM, 46.8:1 reduction ratio with dual-channel AB quadrature encoders [cite: 22, 23] |
| **Motor Driver** | [cite_start]L298N H-Bridge Module [cite: 21] | [cite_start]Dual-channel DC driver supporting up to 2A continuous per motor [cite: 21] |
| **Power Supply** | [cite_start]3 x 18650 Li-ion Batteries [cite: 25] | [cite_start]11.1V nominal package [cite: 25] |
| **Voltage Regulator**| [cite_start]LM2596 DC-DC Buck [cite: 25] | [cite_start]Steps down battery voltage to stable 5V logic power [cite: 25] |
| **Chassis** | [cite_start]Acrylic Plate [cite: 26] | [cite_start]$8 \times 20$ cm frame with 65mm wheels [cite: 26] |
| **Programmer** | [cite_start]ST-LINK V2  | [cite_start]SWD protocol interface for firmware flashing and real-time debugging  |


## ⚙️ Peripherals & STM32 Pin Configuration

[cite_start]The firmware is developed using **STM32CubeIDE** utilizing the **HAL (Hardware Abstraction Layer)**[cite: 8, 11]. Peripherals are tailored as follows:

* **System Core:**
    * [cite_start]**RCC:** High-Speed External (HSE) clock configured with an 8MHz external crystal resonator[cite: 35].
    * [cite_start]**SYS:** Serial Wire Debug (SWD) configuration for ST-LINK communication.
* **Connectivity ($I^2C$):**
    * [cite_start]**I2C1 (PB8 - SCL, PB9 - SDA):** Standard mode at 100 kHz for polling MPU-6050 registers[cite: 32, 33, 34, 48].
* **Timers & PWM Generation:**
    * [cite_start]**TIM3 (PA6 - CH1, PA7 - CH2):** Configured for high-frequency PWM generation (Prescaler: 3, ARR: 999) to drive motor channels smoothly[cite: 38, 39].
    * [cite_start]**TIM2 (PA0/PA1) & TIM4 (PD12/PD13):** Hardware Encoder Mode configured to capture dual-channel quadrature encoder feedback from the left/right wheels[cite: 39, 41].
    * [cite_start]**TIM5 (Interrupt Mode):** Core execution tick running at exactly **200Hz** ($84\text{MHz} / (8399+1) / (49+1)$) creating an absolute 5ms deterministic control window[cite: 41, 44].
* **GPIO Output (Direction Controls):**
    * [cite_start]**PB0, PB1:** Left motor directional configuration[cite: 37].
    * [cite_start]**PE7, PE8:** Right motor directional configuration[cite: 38].


## 🧠 Algorithmic Implementation

### 1. Sensor Fusion: Kalman Filter
[cite_start]Raw data from a gyroscope suffers from low-frequency drift over time, whereas accelerometer readings are heavily corrupted by high-frequency vibration noises from the motors[cite: 54]. [cite_start]The Kalman filter handles this by combining the predictive gyro-rate vector with structural accelerometer angles[cite: 55, 56]:
* [cite_start]**Predict State:** Establishes pitch prediction using angular velocity and updates the error covariance matrix[cite: 55].
* [cite_start]**Measurement Update:** Compares the predicted projection with the static gravity vector to calculate the optimal Kalman gain, producing a clean, chatter-free `robot_angle`[cite: 56, 58].

### 2. Control Strategy: Cascade PID
[cite_start]To prevent the robot from drifting continuously due to geometric tolerances or uneven terrains, a **Dual-Loop (Cascade) PID** controller is deployed[cite: 61]:
OUTER LOOP (Velocity Reg)
           ┌────────────────────────────────────┐
Speed Setpoint │  ┌────────┐         Target Angle   │
───────────────┴─►│ PID_V  ├──────────────────────┐ │
└────────┘                      ▼ │
INNER LOOP (Balance Reg)
┌───────────────────────────────────┐
│  ┌────────┐          Total PWM    │
└─►│ PID_B  ├────────►[Deadzone]───► Motors
└────────┘             ▲
│
Dynamic Angle

* **Inner Loop (Balance Loop):** Executes every 5ms to keep the robot vertical[cite: 59, 61].
    $$\text{balance\_pwm} = K_{p\_bal} \times \text{angle\_error} + K_{d\_bal} \times (0 - \text{gyro\_y\_rate}) + K_{i\_bal} \times \text{angle\_integral}$$
* **Outer Loop (Velocity Loop):** Polls encoder data to evaluate the linear shift[cite: 57, 62]. If a drift trend is captured, it updates the `target_angle` parameter of the inner loop, forcing the physical center of mass to pitch slightly backward/forward to correct position tracking[cite: 63, 64].

### 3. Non-Linear Deadzone Offset
Due to static friction in small metal gearboxes, the motors fail to turn if the raw calculated PWM is too small[cite: 199, 87]. A hard-coded threshold `PWM_DEADZONE = 300.0` is added to any active non-zero output, bypassing sluggish micro-oscillations and boosting system responsiveness[cite: 65, 72, 87].

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
