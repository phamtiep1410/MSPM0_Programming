# Software Architecture for General Power Converter

This document outlines the structure and functions required to develop firmware for a power converter system. The firmware is divided into three layers: Driver Layer, Middleware Layer, and Application Layer.

---

## 1. Driver Layer
The Driver Layer interfaces directly with hardware components. It provides low-level functions to initialize, configure, and control peripherals, abstracting hardware details for higher layers.

### Responsibilities:
- Hardware initialization and configuration.
- Direct control of peripherals (e.g., ADC, PWM, GPIO, UART, SPI, CAN).
- Providing interrupt service routines (ISRs) for handling hardware events.

### Functions in the Driver Layer:
```c
// Peripheral Drivers
void ADC_Init(void);
void ADC_StartConversion(void);
uint16_t ADC_ReadValue(void);

void PWM_Init(void);
void PWM_SetDutyCycle(float dutyCycle); // Duty cycle range: 0.0 (0%) to 1.0 (100%)
void PWM_Start(void);
void PWM_Stop(void);

void GPIO_Init(void);
void GPIO_SetPin(uint8_t pin, uint8_t state); // State: 0 (LOW), 1 (HIGH)
uint8_t GPIO_ReadPin(uint8_t pin);

// Communication Interfaces
void UART_Init(uint32_t baudRate); // Baud rate in bits per second
void UART_SendData(uint8_t *data, uint16_t length);
void UART_ReceiveData(uint8_t *buffer, uint16_t length);

void SPI_Init(void);
void SPI_Transmit(uint8_t *data, uint16_t length);
void SPI_Receive(uint8_t *buffer, uint16_t length);

void CAN_Init(void); // Initializes the CAN peripheral
void CAN_DeInit(void); // Deinitializes the CAN peripheral

// Interrupt Service Routines
void ISR_ADC(void);
void ISR_PWM(void);
void ISR_GPIO(void);

```

## 2. Middleware Layer
The Middleware Layer acts as a bridge between the Driver Layer and the Application Layer. It provides higher-level functionality, such as communication protocols, control algorithms, task scheduling, and data processing.

### Responsibilities:
- Implementing communication protocols (e.g., CAN, Modbus).
- Providing control algorithms for system operation (e.g., PFC and DAB control loops).
- Managing task scheduling and real-time operations.
- Processing data (e.g., filtering, buffering).
- Handling system configurations and fault management.

### Functions in the Middleware Layer:
```c
// Communication Protocols
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length);
void CAN_ReceiveMessage(uint32_t *id, uint8_t *data, uint8_t *length);

void Modbus_Init(void); // Initializes the Modbus protocol
void Modbus_ProcessRequest(uint8_t *request, uint8_t *response);

// Control Algorithms
void PFC_ControlLoop(void); // Totem-Pole PFC control loop
void DAB_ControlLoop(void); // Dual Active Bridge control loop

// Task Scheduler / RTOS
void Scheduler_Init(void);
void Scheduler_AddTask(void (*task)(void), uint32_t interval); // Interval in milliseconds
void Scheduler_Run(void);

// Data Processing
float Filter_Signal(float input); // Example: Low-pass filter
void Buffer_AddData(float data); // Adds data to a circular buffer
float Buffer_GetAverage(void); // Computes the average of buffered data

// System Configurations
void FaultHandler_Init(void);
void FaultHandler_Check(void); // Checks for system faults
void StateMachine_Run(void); // Runs the system state machine
```

## **Control Algorithms**
Control algorithms are used to process system inputs (e.g., sensor data) and generate outputs (e.g., control signals) for various power converter operations. These algorithms are essential for maintaining system stability, efficiency, and performance.

**PI Controller:**
- Purpose: A Proportional-Integral (PI) controller is used for regulating DC quantities, such as voltage or current, in systems like DC-DC converters or Totem-Pole PFC stages.
- Where It Can Be Used:
  - Regulating the output voltage of a DC-DC converter.
  - Controlling the current in a Totem-Pole PFC stage.
```c
typedef struct {
    float kp;       // Proportional gain
    float ki;       // Integral gain
    float integral; // Integral accumulator
    float max;      // Maximum output limit
    float min;      // Minimum output limit
} PI_Controller;

void PI_Controller_Init(PI_Controller *controller, float kp, float ki, float min, float max);
float PI_Controller_Compute(PI_Controller *controller, float error);
```

**PR Controller:**
- Purpose: A Proportional-Resonant (PR) controller is used for regulating AC quantities, such as sinusoidal currents or voltages, in systems like inverters or grid-connected converters.
- Where It Can Be Used:
  - Controlling the AC current in a grid-connected inverter.
  - Regulating the output voltage of an AC power supply.
```c
typedef struct {
    float kp;       // Proportional gain
    float ki;       // Integral gain
    float kr;       // Resonant gain
    float omega;    // Resonant frequency
    float integral; // Integral accumulator
    float prev_error; // Previous error
} PR_Controller;

void PR_Controller_Init(PR_Controller *controller, float kp, float ki, float kr, float omega);
float PR_Controller_Compute(PR_Controller *controller, float error);
```

**TOGI_PLL:**
- Purpose: A Second-Order Generalized Integrator Phase-Locked Loop (TOGI_PLL) is used for estimating the phase and frequency of an AC signal, which is critical for synchronization in grid-connected systems.
- Where It Can Be Used:
  - Synchronizing a grid-connected inverter with the AC grid.
  - Estimating the phase and frequency of an AC signal in power systems.
```c
typedef struct {
    float kp;       // Proportional gain
    float ki;       // Integral gain
    float frequency; // Estimated frequency
    float phase;    // Estimated phase
    float integral; // Integral accumulator
} TOGI_PLL;

void TOGI_PLL_Init(TOGI_PLL *pll, float kp, float ki);
void TOGI_PLL_Update(TOGI_PLL *pll, float input, float dt);
```
  
## **3. Application Layer**
The Application Layer handles system-level initialization, user interfaces, and performance monitoring. It interacts with the Middleware Layer to execute system-level tasks and provide a user-friendly interface.

### Responsibilities:
- System initialization and startup.
- Managing user interfaces (e.g., displaying messages, reading inputs).
- Setting and monitoring system modes.
- Logging and displaying performance metrics.
- Providing APIs for external control and parameter management.
  
### Functions in the Application Layer:

```c
// System Initialization
void System_Init(void); // Initializes the entire system
void System_Start(void); // Starts the system operation

// User Interfaces
void UI_DisplayMessage(const char *message); // Displays a message to the user
void UI_ReadInput(uint8_t *input); // Reads user input (e.g., button press)

// System Modes
void System_SetMode(uint8_t mode); // Sets the system mode
uint8_t System_GetMode(void); // Gets the current system mode

// Performance Monitoring
void Monitor_LogData(void); // Logs system performance data
void Monitor_DisplayPerformance(void); // Displays performance metrics

// APIs for External Control
void API_SetParameter(uint8_t paramId, float value); // Sets a parameter value
float API_GetParameter(uint8_t paramId); // Gets a parameter value
```

**How These Algorithms Fit in the System**
- Driver Layer: Provides raw data from sensors (e.g., ADC values) and actuates hardware (e.g., PWM signals).
- Middleware Layer: Processes the raw data using control algorithms (e.g., PI, PR, TOGI_PLL) to generate control signals.
- Application Layer: Uses the control signals to implement system-level behavior (e.g., regulating output voltage, synchronizing with the grid).

## Summary of Layer Responsibilities

| **Layer**           | **Responsibilities**                                                                 |
|----------------------|---------------------------------------------------------------------------------------|
| **Driver Layer**     | Direct hardware interaction, peripheral initialization, and interrupt handling.       |
| **Middleware Layer** | Protocols, control algorithms, task scheduling, data processing, and fault handling.  |
| **Application Layer**| System-level behavior, user interfaces, performance monitoring, and external APIs.    |

# Header File Description: `My_Controller.h`

This header file defines the implementation of **PI (Proportional-Integral)** and **PR (Proportional-Resonant)** controllers. These controllers are widely used in control systems for regulating DC and AC signals, respectively. The file includes initialization, computation, and reset functions for both controllers.

---

## **1. PI Controller**
The **PI Controller** is used for regulating DC quantities, such as voltage or current, in systems like DC-DC converters or Totem-Pole PFC stages.

### **Structure: `PI_Params_Typedef`**
This structure holds the parameters and state variables for the PI controller.

- **Fields**:
  - `kp`: Proportional gain.
  - `ki`: Integral gain.
  - `Ts`: Sampling time (time interval between successive controller updates).
  - `integral`: Integral term accumulator.
  - `output`: Current output of the PI controller.
  - `output_min`: Minimum output limit.
  - `output_max`: Maximum output limit.
  - `reset_flag`: Reset flag (1 = reset integral term, 0 = normal operation).

### **Functions**
1. **`PI_Params_Init`**:
   - Initializes the `PI_Params_Typedef` structure with the specified parameters.
   - **Parameters**:
     - `PI_Params`: Pointer to the `PI_Params_Typedef` structure.
     - `kp`: Proportional gain.
     - `ki`: Integral gain.
     - `Ts`: Sampling time.
     - `output_min`: Minimum output limit.
     - `output_max`: Maximum output limit.

2. **`PIController`**:
   - Computes the PI control output based on the given error and updates the controller state.
   - **Parameters**:
     - `PI_Params`: Pointer to the `PI_Params_Typedef` structure.
     - `error`: The error signal (setpoint - measured value).
     - `Ts`: Sampling time.
   - **Returns**: The computed control output.

3. **`PIController_Reset`**:
   - Resets the integral term of the PI controller by setting it to zero.
   - **Parameters**:
     - `PI_Params`: Pointer to the `PI_Params_Typedef` structure.

---

## **2. PR Controller**
The **PR Controller** is used for regulating AC quantities, such as sinusoidal currents or voltages, in systems like inverters or grid-connected converters.

### **Structure: `PR_Params_Typedef`**
This structure holds the parameters and state variables for the PR controller.

- **Fields**:
  - `kpr`: Proportional gain.
  - `kir`: Resonant gain.
  - `wc`: Bandwidth of the resonant controller.
  - `w0`: Resonant frequency (e.g., grid frequency in radians per second).
  - `u1`, `u2`: Internal state variables for the resonant controller.
  - `Ts`: Sampling time.
  - `output`: Current output of the PR controller.
  - `output_min`: Minimum output limit.
  - `output_max`: Maximum output limit.
  - `k`: Precomputed gain factor for the resonant controller.
  - `W0Ts`: Precomputed term for the resonant frequency and sampling time.
  - `reset_flag`: Reset flag (1 = reset internal states, 0 = normal operation).

### **Functions**
1. **`PR_Params_Init`**:
   - Initializes the `PR_Params_Typedef` structure with the specified parameters.
   - **Parameters**:
     - `PR_Params`: Pointer to the `PR_Params_Typedef` structure.
     - `kpr`: Proportional gain.
     - `kir`: Resonant gain.
     - `Ts`: Sampling time.
     - `W0`: Resonant frequency.
     - `Wc`: Bandwidth of the resonant controller.
     - `output_max`: Maximum output limit.
     - `output_min`: Minimum output limit.

2. **`PRController`**:
   - Computes the PR control output based on the measured signal and updates the controller state.
   - **Parameters**:
     - `PR_Params`: Pointer to the `PR_Params_Typedef` structure.
     - `measure`: The measured signal (e.g., AC current or voltage).
     - `Ts`: Sampling time.
   - **Returns**: The computed control output.

3. **`PRController_Reset`**:
   - Resets the internal states of the PR controller by setting them to zero.
   - **Parameters**:
     - `PR_Params`: Pointer to the `PR_Params_Typedef` structure.

---

## **Usage**
### **PI Controller**
1. **Initialization**:
   Use `PI_Params_Init` to configure the PI controller parameters.
2. **Computation**:
   Call `PIController` in a control loop to compute the control output based on the error signal.
3. **Reset**:
   Use `PIController_Reset` to reset the integral term when needed (e.g., during system restarts).

### **PR Controller**
1. **Initialization**:
   Use `PR_Params_Init` to configure the PR controller parameters.
2. **Computation**:
   Call `PRController` in a control loop to compute the control output based on the measured signal.
3. **Reset**:
   Use `PRController_Reset` to reset the internal states when needed (e.g., during system restarts).

---

## **Applications**
- **PI Controller**:
  - Regulating DC quantities like voltage or current in DC-DC converters or Totem-Pole PFC stages.
- **PR Controller**:
  - Controlling AC quantities like sinusoidal currents or voltages in grid-connected inverters or AC power supplies.

This header file provides a modular and reusable implementation of PI and PR controllers for various control system applications.

# TOGI PLL Library Documentation

**Author**: The-Tiep Pham  
**Revision**: April 06, 2025  

---

## Overview

This library implements a grid synchronization technique using a **Time-Optimal Generalized Integrator Phase-Locked Loop (TOGI-PLL)**. It provides robust performance under conditions such as:

- **DC offset presence**
- **Frequency deviation**
- **Unbalanced or distorted voltage**

It is inspired by the following publication:

> **Zhang, C. et al. (2018)**  
> *A grid synchronization PLL method based on mixed second- and third-order generalized integrator for DC offset elimination and frequency adaptability*  
> IEEE Journal of Emerging and Selected Topics in Power Electronics, 6(3), pp.1517–1526.

---

## Purpose of the TOGI Library

The goal is to synchronize with a grid signal (e.g., grid voltage) by extracting the **phase angle**, **angular frequency**, and **dq-axis components** while rejecting any **DC offset** in the measurement. This is critical for applications such as:

- Grid-connected inverters
- Synchronization of control systems to AC mains
- Frequency estimation in digital controllers

---

## Data Structure: `TOGI_PLL_Params_Typedef`

This structure holds all the dynamic variables of the PLL:

| Field            | Description |
|------------------|-------------|
| `Vpk`            | Peak detected voltage amplitude |
| `Omega`          | Estimated grid angular frequency (rad/s) |
| `theta`          | Estimated grid phase angle (radians) |
| `V_Offset`       | Estimated DC component of the input |
| `Val`, `Vbe`     | Generalized integrator outputs (α-β) |
| `Val_Real`, `Vbe_Real` | Corrected α-β voltages (DC-free) |
| `Vd`, `Vq`       | d-q voltages (used for PLL regulation) |
| `Sine`, `Cose`   | Sine and cosine of the phase angle |
| `TOGI_Ready_Flag`| Indicates if PLL is locked and ready (1: true, 0: false) |

---

## Function: `void TOGI_PLL_Params_Init(...)`

### **Purpose**  
Initializes all fields in the `TOGI_PLL_Params_Typedef` structure to default values (mostly zeros). This function should be called once before starting the PLL loop.

### **Input**  
- `TOGI_PLL_Params`: Pointer to the structure that stores the internal states.

### **Output**  
- The structure is updated with default values.

---

## Function: `void TOGI_PLL(...)`

### **Purpose**  
Executes one iteration of the TOGI PLL. It processes the input voltage to estimate the grid phase, frequency, and voltage components in α-β and d-q frames.

### **Inputs**  
| Name         | Type   | Description |
|--------------|--------|-------------|
| `TOGI_PLL_Params` | `TOGI_PLL_Params_Typedef*` | Pointer to the state structure |
| `Vac`        | `float`| Instantaneous AC input voltage |
| `Ts`         | `float`| Sampling period (seconds) |
| `W0`         | `float`| Nominal angular frequency (e.g., 2π×50 = 314.16 rad/s) |
| `kp_PLL`     | `float`| Proportional gain for PI controller |
| `ki_PLL`     | `float`| Integral gain for PI controller |

### **Outputs**  
- Updates the internal state structure:
  - `Omega`, `theta` are updated to reflect phase and frequency.
  - `Vd`, `Vq` are computed for synchronization control.
  - `TOGI_Ready_Flag` is set to 1 if the PLL is stable.

---

### **How It Works**

1. **DC Offset Elimination**  
   - The voltage input is filtered through second and third-order integrators to isolate the DC component (`V_Offset`) and obtain α-β components (`Val`, `Vbe`).

2. **α-β to d-q Transformation**  
   - Using the estimated `theta`, the algorithm projects α-β voltages into the rotating d-q frame.

3. **PLL PI Controller**  
   - The `Vd` component is fed into a PI controller.
   - `Omega` is adjusted based on the controller output, tracking the grid frequency.
   - The phase `theta` is incremented based on `Omega`.

4. **Lock Detection**  
   - If `Vd` is near zero and `-Vq` exceeds a threshold, it means the PLL has locked onto the signal, and `TOGI_Ready_Flag` is set.

5. **Phase Wrap-around**  
   - `theta` is wrapped to [0, 2π] after each update.

---

## Suggested Parameters

| Parameter | Typical Value | Notes |
|----------|---------------|-------|
| `Ts`     | `1e-4` s       | For 10 kHz control frequency |
| `W0`     | `314.16` rad/s | 50 Hz system |
| `kp_PLL` | `10.0`         | Proportional gain (tune based on bandwidth) |
| `ki_PLL` | `1000.0`       | Integral gain (to eliminate steady-state error) |

---

## Applications

- Digital grid synchronization
- AC signal conditioning
- Frequency/phase tracking
- Real-time embedded control (e.g., STM32, TMS320, etc.)

---

## Example Usage (Pseudocode)

```c
TOGI_PLL_Params_Typedef pll_params;
TOGI_PLL_Params_Init(&pll_params);

while (1) {
    float Vac = read_adc_voltage();
    TOGI_PLL(&pll_params, Vac, 1e-4, 314.16, 10.0, 1000.0);

    if (pll_params.TOGI_Ready_Flag) {
        use_theta(pll_params.theta);  // Feed to control loop
    }
}
```

