# Knife Sharpening Trainer

Knife Sharpening Trainer is an embedded hardware/software project for measuring a knife's sharpening angle in real time and giving the user immediate feedback while sharpening.

The current prototype centers on a 6-DOF IMU attached to the knife and an ESP32-class microcontroller running Rust firmware. The embedded system estimates orientation with a Mahony-style attitude filter, computes sharpening angle geometrically from plane normals, and will expose telemetry over Bluetooth Low Energy (BLE).

The immediate objective is not a polished product. It is to prove that the measurement approach is accurate, stable, responsive, and practical during real sharpening motion.

## Current Development Strategy

Development is intentionally staged so that the core mathematics can be implemented and tested before hardware arrives.

The V1 path is:

~~~text
Mathematical model
      ↓
Host-testable Rust math/core crates
      ↓
Simulation + unit tests
      ↓
ESP32 + IMU hardware bring-up
      ↓
Embedded Rust firmware
      ↓
BLE telemetry
      ↓
Laptop diagnostic/test application
      ↓
Physical validation and tuning
      ↓
Swift/SwiftUI companion app later
~~~

For V1, the laptop will act as the primary development and diagnostic client for BLE data from the ESP32. A Swift/SwiftUI iPhone app is intentionally deferred until the embedded implementation and BLE protocol are substantially stabilized.

## Mathematical Approach

The estimator and the user-facing sharpening-angle calculation are treated as separate concerns.

The IMU provides accelerometer and gyroscope measurements. A 6-DOF Mahony attitude filter maintains the device orientation as a unit quaternion using the convention:

~~~text
[w, x, y, z]
~~~

The quaternion maps the current IMU/body frame into the fixed calibration frame. Gyroscope values are represented internally in radians per second, and internal angle values are represented in radians.

The sharpening angle is **not** extracted as Euler pitch or roll. Instead, the current blade-plane normal is rotated into the calibration frame and compared geometrically against the calibrated reference-plane normal. This avoids Euler-order coupling and better matches the physical sharpening problem, where the knife may intentionally roll or change azimuth while maintaining the same blade-to-stone angle.

The current mathematical specification is stored at:

~~~text
resources/knife_sharpener_mahony_mathematical_schematic.html
~~~

That document is the implementation contract for coordinate conventions, calibration behavior, Mahony update order, adaptive accelerometer weighting, tuning parameters, numerical safeguards, and pre-hardware sanity tests.

## V1 Software Architecture

The expected Rust workspace will evolve toward a structure similar to:

~~~text
knife-sharpening-trainer/
├── Cargo.toml
├── AGENTS.md
├── README.md
├── resources/
│   └── knife_sharpener_mahony_mathematical_schematic.html
├── crates/
│   ├── sharpener-math/
│   ├── sharpener-core/
│   ├── sharpener-protocol/
│   ├── sharpener-sim/
│   ├── sharpener-firmware/
│   └── sharpener-desktop/
├── hardware/
├── docs/
├── recordings/
└── tests/
~~~

The exact layout may change as the ESP32 Rust ecosystem and selected hardware dictate, but the architectural boundaries should remain explicit.

### `sharpener-math`

Pure, host-testable mathematical primitives such as:

- 3D vector operations
- Quaternion normalization
- Quaternion conjugation
- Hamilton quaternion multiplication
- Vector rotation between body and calibration frames

This crate should contain no ESP32 dependencies.

### `sharpener-core`

The hardware-independent sharpening algorithm:

- Bench-rest calibration
- Startup gyro-bias estimation
- Mahony attitude update
- Adaptive accelerometer confidence weighting
- Residual gyro-bias learning
- Geometric sharpening-angle calculation
- Target-angle comparison
- Feedback state

The core crate should accept normalized `ImuSample` inputs rather than talking to hardware directly.

### `sharpener-protocol`

The communications contract shared by the ESP32 firmware and development clients.

Expected concepts include:

- Device commands
- Calibration commands and state
- Target-angle commands
- Device status
- Telemetry frames
- Diagnostic fields
- Protocol versioning

The BLE protocol should remain explicit and documented rather than being defined implicitly by either firmware or UI code.

### `sharpener-sim`

Synthetic IMU generation and algorithm testing before hardware is available.

Important simulation cases include:

- Identity orientation after calibration
- Known 15° tilt
- Rotation about the blade normal
- Constant inclination with changing azimuth
- Gyroscope bias
- Accelerometer noise
- Linear-acceleration disturbances
- Accelerometer rejection and recovery
- Low-confidence integral freeze

### `sharpener-firmware`

ESP32-specific Rust code responsible for:

- IMU driver and sampling
- Sensor unit conversion
- Timing
- Calibration control
- Invoking `sharpener-core`
- BLE communications
- Audio feedback
- Embedded logging and diagnostics

Hardware-specific code should remain isolated from the measurement mathematics wherever practical.

### `sharpener-desktop`

A simple laptop diagnostic application used during V1 hardware development.

Its purpose is engineering visibility rather than polished UX. It should be able to:

- Discover/connect to the ESP32 over BLE
- Start calibration
- Set or capture target angle
- Display live sharpening angle
- Display raw accelerometer and gyroscope data
- Display the attitude quaternion
- Display Mahony innovation values
- Display residual gyro bias
- Display accelerometer residual `rho`
- Display candidate/current accelerometer confidence
- Display sample timing / `dt`
- Log telemetry to CSV for offline analysis

CSV logging is considered a first-class V1 diagnostic feature because filter gains and confidence thresholds will be tuned from recorded physical experiments.

## Hardware Direction

The initial prototype uses:

- An ESP32-class development board suitable for Rust embedded development
- A 6-DOF IMU with accelerometer and gyroscope
- BLE for communication
- USB for initial programming/power/debugging

A magnetometer is not required for the initial design because absolute heading around gravity is not needed for the sharpening-angle calculation.

The initial calibration method assumes the blade is placed flat on a level reference surface. This allows the stationary accelerometer direction to identify the blade/reference-plane normal even if the IMU is not mounted perfectly square to the blade.

## Testing Philosophy

The project should validate mathematics independently of hardware wherever possible.

Host-side tests should cover:

- Vector and quaternion primitives
- Coordinate-frame directionality
- Quaternion multiplication order
- Calibration invariants
- Sharpening-angle geometry
- Mahony update behavior
- Accelerometer confidence weighting
- Numerical safeguards
- Protocol encoding/decoding

Hardware testing should then focus on the things simulation cannot prove:

- Sensor quality
- Timing jitter
- Real gyro bias and drift
- Linear acceleration during sharpening
- Mechanical mounting repeatability
- BLE reliability
- End-to-end latency

Physical validation should use known reference angles and recorded experiments to quantify absolute error, repeatability, drift, response time, and noise.

## Engineering Principles

- Rust is the required embedded implementation language.
- Keep hardware-specific code separate from host-testable algorithm code.
- Use SI units internally where practical.
- Keep coordinate systems and frame transformations explicit.
- Treat calibration as part of the measurement system, not as an afterthought.
- Do not equate generic Euler pitch with sharpening angle.
- Prefer measurable, incremental prototypes over premature product complexity.
- Keep BLE as a documented contract between independently buildable components.
- Log first, then tune.
- Do not add cloud infrastructure, accounts, databases, or similar product infrastructure until the physical measurement concept is proven.

## Project Guidance for Coding Agents

See [`AGENTS.md`](./AGENTS.md) before making changes. It contains the broader project architecture, development constraints, hardware assumptions, testing strategy, and agent-specific guidance.

## Status

The project is currently in the pre-hardware / algorithm implementation phase. The mathematical model has been defined, and the next engineering milestone is to establish the Rust workspace and implement/test the math and orientation core before integrating the real ESP32 and IMU.
