# Electrical Architecture

![main_electronics_scheme.drawio.png](../assets/images/main_electronics_scheme.drawio.png)

---

## SS
The system has to primary tasks: to fly and to walk. Due to how different these two system operate - it was decided to take a "split-brain" approach.

Flying subsystem consistent 

Walking subsystem 

## Power Distribution

The system is powered by a single **850mAh 4S 95C LiPo battery**. Power is routed through an XT60 pigtail and split via a three-way junction to the SpeedyBee ESC, Matek Micro UBEC, and Hobbywing 25A UBEC. This parallel distribution isolates subsystems with fundamentally different voltage and current requirements.

### Logic Power
The **Matek Micro UBEC** steps down the primary battery voltage to a regulated 5V, supplying 1.5A continuous (2.5A peak). This provides the dedicated, low-current logic power required by the Raspberry Pi Zero processor.

### Leg Power
The **Hobbywing 25A UBEC** steps down battery voltage to 6.0V/7.4V to power the 12 Hiwonder servos. These servos possess a collective theoretical stall current of 30A (12 servos × 2.5A). A standard, lower-rated UBEC would fail under the load of demanding maneuvers.

Because standard 2.54mm pin headers are generally only rated for ~3A of continuous current, drawing 25A–50A through a single port would result in thermal failure. To safely manage this, the UBEC's 8 output ports are wired in parallel to form a custom high-current distribution bus. This architecture shares the load, reducing the maximum current draw per individual pin to safe operating margins (3.75A per pin under a 30A peak load) before distributing it out to the 12 servos.

The ground rail follows the same parallel bus architecture. In addition to the 12 servo ground returns, a dedicated synchronization wire connects this high-current ground bus directly to a GND pin on the PCA9685 to ensure a common zero-volt reference across the logic and actuation systems.

### Flight Power
The **SpeedyBee ESC** receives raw battery voltage to directly drive the four FlyFishRC Flash 1804 3500KV motors. The ESC also passes this raw battery voltage up to the SpeedyBee Flight Controller mounted on top of it, which regulates its own internal logic power via an integrated voltage regulator.

### Auxiliary Power Routing
* **PCA9685 (PWM Driver):** Receives 3.3V logic power from the Raspberry Pi. To prevent  thermal failure of the board's internal copper traces, the PCA9685's terminal block is deliberately bypassed. The 30A servo actuation current is routed entirely through the external parallel bus.
* **Radio Link:** The Radiomaster RP2 V2 TCXO receiver draws its 5V logic power directly from a dedicated 5V pad on the SpeedyBee FC.
* **Pi Cam:** Raspberry Pi Camera V2 is connected to Raspberry Pi through CSI cable which is used for both data and power transfer.

---

## Communications & Logic Architecture

Data routing across the system is strictly delineated by subsystem to ensure the Linux-based environment of the Raspberry Pi does not interrupt the real-time processing of the flight controller. The communication topologies are divided into the following discrete channels:

### Vision System (Raspberry Pi & Pi Camera)
* **Interface:** MIPI CSI-2
* **Implementation:** Connected via a dedicated flat ribbon cable.
* **Function:** Provides a high-bandwidth, direct memory access (DMA) pipeline for raw video data transfer from the camera module to the Pi Zero 2 W, enabling low-latency computer vision and automation processing.

### Kinematics Control Loop (Raspberry Pi & PCA9685)
* **Interface:** I2C Bus
* **Implementation:** Pi SDA connects to PCA9685 SDA; Pi SCL connects to PCA9685 SCL.
* **Function:** The Pi does complex calculations in ROS and transmits the resulting multi-channel positioning commands over the I2C bus to the PCA9685. The PCA9685 then generates the precise hardware-level PWM signals required to actuate the 12 Hiwonder servos.

### Companion Computer Link (Raspberry Pi & SpeedyBee FC)
* **Interface:** Bi-directional Asynchronous Serial (UART)
* **Implementation:** Crossed logic lines; Pi TX connects to FC RX, and Pi RX connects to FC TX.
* **Function:** The Raspberry Pi acts as a "Companion Computer" to the Flight Controller. This bi-directional link serves a dual purpose:
    1. **Telemetry Stream:** The Pi receives high-frequency IMU data from the FC.
    2. **Offboard Control:** Pi transmits high-level autonomous flight directives derived from its vision processing. The FC receives these commands and handles the critical, low-level PID motor mixing required to execute them safely.

### **Manual Pilot Override (SpeedyBee** FC & Radio Link)
* **Interface:** Serial (UART)
* **Implementation:** Radiomaster RP2 V2 TCXO receiver wired to the FC's TX and RX pads.
* **Function:** Provides a direct, hardware-level pilot override and fail-safe mechanism. Because it communicates directly with the Flight Controller, manual flight control and emergency motor-kill switches remain fully operational even in the event of a Raspberry Pi OS crash or automation loop failure.

---

## Pinout Tables
This will be added at later point.
