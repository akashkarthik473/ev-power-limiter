# Closed Loop Power Limiting Control System

A C-based implementation of a power limiting control system for electric motors, featuring multiple control strategies including PID control, lookup tables, and mechanical equations.

## Overview

This project implements a power limiting control system that can maintain motor power within specified limits using different control strategies:

1. **Torque Equation Method**: Uses the mechanical power equation (T = P*9549/rpm) to calculate torque limits
2. **Power PID Method**: Directly controls power using a PID controller
3. **Lookup Table (LUT) Method**: Uses a pre-calculated lookup table for torque limits
4. **Combined Method**: (Placeholder for future implementation)

The system includes anti-windup protection, saturation limits, and fallback mechanisms for out-of-range conditions.

## Features

- Multiple control strategies for power limiting
- PID controller with anti-windup protection
- Lookup table-based control with bilinear interpolation
- Mechanical equation-based control
- Configurable power limits and thresholds
- Motor controller simulation for testing
- Real-time status monitoring and debugging

## Requirements

- C compiler (GCC recommended)
- Make or CMake (for building)
- Standard C library
- Math library (-lm)

## Building

```bash
# Using GCC
gcc -o power_limit_test main.c PID.c powerLimit.c -lm

# Using Make (if Makefile is present)
make
```

## Usage

The main test program (`main.c`) demonstrates the power limiting system:

```c
// Initialize motor controller
MotorController mcm = {
    .motorRPM = 3000,        // Initial speed
    .power = 65000,          // Initial power (65kW)
    .dcVoltage = 360,        // DC bus voltage
    .dcCurrent = 0,          // DC bus current
    .commandedTorque = 2310, // Initial torque (231.0 Nm)
    .plStatus = 0            // Power limiting disabled
};

// Initialize power limiter
PowerLimit* pl = POWERLIMIT_new();
pl->plMode = 1;              // Use torque equation method
pl->plTargetPower = 80;      // Target power limit (80kW)
```

### Control Modes

1. **Mode 1**: Torque Equation Method
   - Uses mechanical power equation
   - Good for steady-state operation
   - Simple implementation

2. **Mode 2**: Power PID Method
   - Direct power control
   - Better for dynamic conditions
   - Includes anti-windup

3. **Mode 3**: LUT Method
   - Pre-calculated torque limits
   - Fast execution
   - Includes bilinear interpolation

4. **Mode 4**: Combined Method
   - Placeholder for future implementation
   - Could combine multiple strategies

## Implementation Details

### PID Controller

The PID controller (`PID.h`, `PID.c`) implements:
- Proportional, Integral, and Derivative control
- Anti-windup protection
- Output saturation
- Configurable gains and limits

### Power Limiter

The power limiter (`powerLimit.h`, `powerLimit.c`) provides:
- Multiple control strategies
- Lookup table management
- Torque command calculation
- Status monitoring

### Motor Controller Interface

The motor controller interface (`main.c`) includes:
- Motor state management
- Power and torque calculations
- Status reporting
- Simulation capabilities

## Testing

The test program (`main.c`) simulates motor operation with:
- Fixed motor speed (3000 RPM)
- Initial power of 65kW
- Target power limit of 80kW
- Real-time status reporting
- PID controller monitoring

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
