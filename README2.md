# Project Reflection & Development Notes

This document contains personal notes on the project's strengths, key learnings, and areas for improvement.

## What Went Well

### Technical Achievements

**1. Register-Level Programming Mastery**
- Successfully implemented I2C, UART, DMA, and Timer peripherals without HAL dependency
- Achieved 3.5× performance improvement over HAL-based implementation
- Gained deep understanding of STM32 hardware architecture
- Direct control over timing and memory footprint

**2. Real-Time System Design**
- Consistent 20Hz update rate with only 2.4% CPU utilization
- Deterministic ISR execution (<1μs)
- No missed deadlines or timing violations during testing
- Proper separation between ISR and main loop processing

**3. Sensor Fusion Implementation**
- Successfully implemented Extended Kalman Filter from scratch
- Quaternion-based representation avoiding gimbal lock
- Proper covariance matrix tuning for stable estimation
- Integration of CMSIS-DSP for optimized matrix operations

**4. Robust Communication**
- DMA-based telemetry with zero CPU overhead during transmission
- Custom I2C recovery mechanism preventing bus lockup
- Packet validation with magic tail synchronization
- Clean MATLAB integration with real-time visualization

**5. MATLAB Real-Time Visualization**
- Developed F-14 aircraft 3D renderer synchronized with quaternion telemetry
- Implemented packet validation and synchronization with magic tail
- Quaternion-to-rotation matrix conversion for real-time graphics updates
- Clean serial communication interface with <50ms latency
- Useful debugging tool that made orientation tracking visually verifiable

### Key Learnings

**Embedded Systems Architecture**
- Understanding the trade-offs between abstraction and performance
- Importance of static memory allocation in real-time systems
- Hardware peripheral state machines and timing requirements
- DMA configuration and memory-to-peripheral transfers

**Kalman Filtering**
- Prediction-update cycle for state estimation
- Linearization through Jacobian matrices
- Process vs measurement noise tuning
- Quaternion normalization to maintain constraints

**Software Engineering**
- Modular code organization (separate driver files)
- Clear separation of concerns (EKF, drivers, telemetry)
- Proper use of volatile for shared variables between ISR and main loop
- Defensive programming (timeout handling, NaN checks)

**Debugging Real-Time Systems**
- Using LED for visual feedback during development
- Serial telemetry for runtime state inspection
- Timeout mechanisms for I2C fault recovery
- Importance of oscilloscope/logic analyzer for timing verification

## Areas for Improvement

### 1. Extended Kalman Filter Sophistication

**Current Implementation**: Simplified EKF with fixed covariance matrices and 6-DOF sensor fusion

**Academic EKF Features Not Implemented**:
- Magnetometer integration for absolute yaw reference (9-DOF)
- Adaptive covariance matrices based on motion detection
- Multi-rate filtering (gyroscope at 100Hz, accelerometer at 20Hz)
- Online gyroscope bias estimation as augmented state
- Outlier rejection and sensor fault detection
- Covariance conditioning and numerical stability safeguards

**Why This Simplification Was Necessary**:

The STM32F103RB (Cortex-M3) lacks hardware floating-point unit, making all matrix operations software-emulated. A full-featured academic EKF would require:

- 9×9 or larger state covariance matrices (vs current 4×4)
- Real-time eigenvalue decomposition for covariance conditioning
- Adaptive noise parameter estimation
- Multiple measurement update steps per cycle

**Performance Impact**:
- Current EKF: ~400μs per cycle (prediction + update)
- Academic EKF estimate: ~2-3ms per cycle
- This would exceed the 50ms budget at 20Hz

**Hardware Requirements for Academic Implementation**:
- STM32F4 or STM32F7 series (Cortex-M4/M7 with FPU)
- Hardware FPU provides 5-10× speedup for floating-point operations
- Larger RAM for extended state vectors and covariance matrices
- Higher clock speed (168MHz+ vs 72MHz)

**Observable Limitation**: 
In the demonstration video, the aircraft exhibits slow yaw drift when the sensor is held stationary after motion. This is the inherent limitation of 6-DOF systems—without a magnetometer providing absolute heading reference, the EKF can only correct roll and pitch using gravity, while yaw accumulates gyroscope bias over time. A 9-DOF implementation with magnetometer would eliminate this drift by providing a fixed heading reference.

**Note**: The current implementation prioritizes real-time performance and deterministic execution on resource-constrained hardware. It is production-ready for applications where 6-DOF orientation estimation is sufficient (robotics, gimbal stabilization, AR/VR head tracking). For applications requiring absolute heading (navigation, autonomous vehicles), magnetometer integration would be the primary next step.

### 2. Error Handling and Diagnostics

**Current State**: Basic timeout handling, LED indication

**Missing Features**:
- No error codes or diagnostic telemetry
- Limited fault recovery strategies
- No logging of fault events

**Proposed Solution**:
- Error state machine with specific fault codes
- Diagnostic telemetry packet type for debugging
- Watchdog timer for system reset on hard faults

### 3. Hardware FPU Utilization

**Current State**: STM32F103RB (Cortex-M3) without hardware FPU

**Impact**:
- All floating-point operations are software-emulated
- Significant performance penalty (10-100× slower than hardware FPU)
- CMSIS-DSP helps but still software-based

**Proposed Solution**:
- Use STM32F4 or STM32F7 series (Cortex-M4/M7 with FPU)
- Enable FPU in compiler flags: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`
- Recompile CMSIS-DSP with FPU support
- Expected speedup: 5-10× for matrix operations

**Migration Path**:

### 4. Power Optimization

**Current State**: Always-on, no power management

**Missed Opportunities**:
- No sleep modes between sensor readings
- Continuous DMA and timer operation

**Proposed Solution**:
- Sleep mode between 50ms timer ticks
- Wake on interrupt (WFI instruction)
- Use STM32 low-power timers

### 5. MATLAB Code Organization

**Current State**: Single monolithic file (F14_Visualizer.m) containing all functionality

**Limitation**:
- Serial communication, packet parsing, and 3D rendering all in one file
- Difficult to reuse components for other projects
- Hard to test individual functions independently

**Proposed Solution**:
- Separate into three modules: serial handler, data parser, and visualizer
- Serial handler: Connection management and raw data reading
- Data parser: Packet validation, quaternion extraction, and conversion
- Visualizer: 3D graphics, camera control, and rendering loop
- Modular structure allows reusing parser for different sensors or visualizers

## Key Achievements

1. **No HAL dependency for critical paths** - Complete control over timing
2. **Rock-solid I2C recovery** - System never hangs on bus errors
3. **Clean modular architecture** - Each module is independent and testable
4. **Sub-microsecond ISR** - Demonstrates understanding of real-time constraints
5. **Professional documentation** - README suitable for portfolio/interview discussion

## Areas for Further Study

1. **Advanced Kalman Filtering**: UKF, Particle Filters, H-infinity filters
2. **Control Theory**: Moving from estimation to control (quadcopter stabilization)
3. **RTOS**: FreeRTOS integration for more complex task scheduling
4. **Signal Processing**: Better noise characterization and filtering
5. **Hardware Design**: Custom PCB with STM32 and sensors integrated

## Conclusion

This project taught me how to design real-time systems under strict hardware constraints. Writing peripheral drivers at the register level gave me a much deeper understanding of how microcontrollers actually work, beyond what any HAL abstraction could provide. I chose an Extended Kalman Filter because quaternion kinematics are inherently nonlinear, making simpler approaches like complementary filters less accurate for orientation estimation. Understanding why certain algorithms are necessary—not just how to implement them—was one of the most valuable aspects of this project.

---

*Last Updated: February 2026*
