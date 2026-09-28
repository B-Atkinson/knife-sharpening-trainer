# AGENTS.md

## Project: Knife Sharpener

### 1. Project Overview

Knife Sharpener is a hardware/software project for building a smart
knife-sharpening angle sensor.

The core concept is to attach or integrate an inertial measurement unit
(IMU) with a small embedded controller so the device can determine the
orientation of a knife during sharpening and provide the user with
real-time feedback about the sharpening angle.

The project is intentionally being developed as a practical embedded
Rust project. The embedded firmware should be written in Rust rather
than C/C++, both because Rust is the desired production direction and
because the project is intended to provide hands-on Rust embedded
experience.

The initial product should prioritize proving the measurement concept
and producing a reliable prototype over industrial design or feature
breadth.

------------------------------------------------------------------------

## 2. High-Level Goals

The first prototype should establish these capabilities in order:

1.  Read reliable orientation data from an IMU.
2.  Convert IMU measurements into a useful knife/sharpening angle.
3.  Apply appropriate filtering/sensor-fusion techniques so the
    displayed angle is stable without excessive latency.
4.  Run the measurement system reliably on an ESP32.
5.  Transmit angle/status data wirelessly to a companion device.
6.  Build a Swift/SwiftUI companion application that displays the
    measured angle clearly.
7.  Add calibration and configuration.
8.  Iterate on mechanical mounting, measurement accuracy, ergonomics,
    and eventual productization.

Do not prematurely optimize for a production enclosure, cloud
infrastructure, authentication, or a large application architecture. The
first milestone is a technically credible working prototype.

------------------------------------------------------------------------

## 3. Initial System Architecture

The intended architecture has three primary layers:

\`\`\` text
┌──────────────────────────────┐
│       Knife / Sharpening     │
│                              │
│   IMU measures orientation   │
└──────────────┬───────────────┘
               │
               │ sensor data
               ▼
┌──────────────────────────────┐
│           ESP32              │
│                              │
│  Rust embedded firmware      │
│  ├─ IMU driver               │
│  ├─ Sensor acquisition       │
│  ├─ Filtering / fusion       │
│  ├─ Angle calculation        │
│  ├─ Calibration              │
│  ├─ Device state             │
│  └─ BLE communications       │
└──────────────┬───────────────┘
               │
               │ Bluetooth Low Energy
               ▼
┌──────────────────────────────┐
│      Apple Companion App     │
│                              │
│      Swift / SwiftUI         │
│  ├─ BLE connection           │
│  ├─ Live angle display       │
│  ├─ Device status            │
│  ├─ Calibration              │
│  └─ Configuration            │
└──────────────────────────────┘
\`\`\`

Keep the embedded and application layers decoupled. The ESP32 should
expose a small, explicit communications protocol rather than leaking
internal implementation details to the application.

------------------------------------------------------------------------

## 4. Hardware Architecture

### 4.1 Primary MCU

The prototype uses an ESP32-class microcontroller.

The current direction is specifically an ESP32 variant suitable for Rust
embedded development. Select hardware with good Rust ecosystem support,
appropriate BLE capabilities, sufficient GPIO/I2C/SPI resources, and
readily available development boards.

The ESP32 is responsible for:

-   IMU communication
-   Sensor sampling
-   Orientation/angle calculation
-   Filtering
-   Calibration
-   Device state
-   BLE communications
-   Basic power-management behavior as appropriate

### 4.2 IMU

The primary sensor is an IMU (Inertial Measurement Unit).

The IMU should provide at minimum:

-   Accelerometer data
-   Gyroscope data

A magnetometer is not required for the initial prototype unless testing
demonstrates that it provides meaningful benefit. The initial objective
is knife tilt/sharpening-angle measurement rather than absolute compass
heading.

Likely communication interfaces:

-   I2C (Inter-Integrated Circuit) is preferred for simplicity during
    prototyping.
-   SPI may be considered if sensor bandwidth, reliability, or driver
    support makes it preferable.

The firmware should abstract the IMU behind a sensor interface so the
exact IMU can be changed without rewriting the entire application.

### 4.3 Mechanical/Sensor Considerations

The physical mounting of the IMU matters as much as the software.

The prototype should establish:

-   A repeatable relationship between the sensor coordinate system and
    the knife/blade geometry.
-   A mechanically stable sensor mount.
-   A known reference orientation for calibration.
-   Minimal mechanical play between the sensor and knife.
-   A practical way to attach/detach the sensor from a knife or
    sharpening setup.

Do not assume that mathematically perfect IMU orientation automatically
produces a useful sharpening angle. Mechanical alignment and calibration
are first-class parts of the measurement system.

### 4.4 Prototype Hardware

The initial prototype should include enough supporting hardware to
independently test:

-   ESP32
-   IMU
-   USB programming/power
-   Breadboard/prototyping connections
-   Appropriate jumper wires/connectors
-   Basic power infrastructure
-   Any required voltage regulation/level shifting
-   A physical method of mounting the sensor

The initial BOM should identify:

-   Part name
-   Quantity
-   Vendor/source
-   Notes
-   Relevant specifications or rationale

The project should favor readily obtainable parts from reputable
electronics suppliers.

------------------------------------------------------------------------

## 5. Embedded Software: Rust

### 5.1 Language Requirement

**ESP32 firmware should be implemented in Rust.**

Do not switch the embedded implementation to C/C++ merely because that
is more common in ESP32 examples.

The project is explicitly intended to build practical Rust embedded
experience.

### 5.2 Rust Responsibilities

The Rust firmware should own the hardware-facing and real-time portions
of the system:

\`\`\` text
Hardware
   │
   ▼
IMU driver
   │
   ▼
Raw sensor samples
   │
   ▼
Calibration
   │
   ▼
Filtering / sensor fusion
   │
   ▼
Orientation estimate
   │
   ▼
Sharpening-angle calculation
   │
   ▼
BLE telemetry
\`\`\`

Potential module organization:

\`\`\` text
firmware/
├── src/
│   ├── main.rs
│   ├── imu/
│   │   ├── mod.rs
│   │   └── driver.rs
│   ├── orientation/
│   │   ├── mod.rs
│   │   ├── filter.rs
│   │   └── angle.rs
│   ├── calibration/
│   │   └── mod.rs
│   ├── ble/
│   │   └── mod.rs
│   └── device/
│       └── mod.rs
└── Cargo.toml
\`\`\`

This is a suggested organization, not a rigid requirement. Adapt it to
the actual Rust ecosystem and board support package.

### 5.3 Rust Ecosystem

Prefer mature, actively maintained Rust embedded crates and the
appropriate ESP32 HAL/SDK ecosystem.

Use \`no_std\` where appropriate for the embedded target.

Likely concepts include:

-   HAL (Hardware Abstraction Layer)
-   GPIO
-   I2C/SPI
-   timers
-   interrupts
-   async tasks where useful
-   embedded-hal traits
-   BLE stack
-   sensor driver crates
-   defmt/logging where supported

Do not add large dependencies without a clear reason.

The firmware should be designed so hardware-specific code is isolated
from angle-estimation logic wherever practical.

### 5.4 Concurrency

The firmware will likely have at least two conceptual workloads:

1.  Sensor acquisition / angle calculation.
2.  BLE communications.

Use the simplest concurrency model that reliably meets timing
requirements.

Possible approaches include an async embedded runtime or a
straightforward periodic loop with nonblocking/event-driven BLE
handling, depending on the selected ESP32 platform and ecosystem.

Do not introduce RTOS-style complexity unless it provides a measurable
benefit.

------------------------------------------------------------------------

## 6. Angle Measurement

The fundamental technical problem is determining the knife's angle
relative to the sharpening reference plane.

An accelerometer provides a gravity reference when the knife is
relatively stationary. A gyroscope provides short-term angular-rate
information and helps smooth dynamic movement. Sensor fusion can combine
them to produce a more stable orientation estimate.

The implementation should distinguish between:

-   Raw accelerometer measurements
-   Raw gyroscope measurements
-   Estimated orientation
-   Device/knife reference orientation
-   Final displayed sharpening angle

The exact mathematical approach should be selected experimentally.

Potential algorithms include:

-   Complementary filter
-   Kalman-family filter
-   Madgwick filter
-   Mahony filter

Start with the simplest method that produces acceptable results. A
complementary filter may be sufficient for the first prototype.

### Important distinction

The application should not necessarily display a generic "device pitch"
value and call it sharpening angle.

Instead, define a coordinate system and explicitly model:

\`\`\` text
IMU orientation
      +
sensor-to-knife mounting transform
      +
calibration/reference orientation
      =
knife orientation
      +
sharpening reference geometry
      =
displayed sharpening angle
\`\`\`

This separation will make calibration and mechanical iteration
substantially easier.

------------------------------------------------------------------------

## 7. Calibration

Calibration is a required part of the design, not an afterthought.

The system should eventually support a user calibration procedure that
establishes the relationship between:

-   IMU coordinate frame
-   Device housing
-   Knife/blade
-   Sharpening surface/reference plane

The initial prototype can use a simple calibration workflow.

Example conceptual process:

1.  Place the device/knife in a known reference orientation.
2.  Allow the system to collect several samples.
3.  Estimate the reference orientation.
4.  Store the calibration offset.
5.  Express subsequent measurements relative to that reference.

Avoid hard-coding orientation offsets into the firmware.

Calibration data should be represented explicitly and eventually
persisted in nonvolatile storage.

------------------------------------------------------------------------

## 8. Bluetooth Low Energy Protocol

BLE (Bluetooth Low Energy) is the intended communications mechanism
between the ESP32 and the Apple companion app.

The protocol should remain intentionally small.

The ESP32 should expose services/characteristics for concepts such as:

-   Current angle
-   Measurement state
-   Device status
-   Battery level, if applicable
-   Calibration state
-   Configuration
-   Firmware/device information

A conceptual telemetry packet might contain:

\`\`\` text
timestamp / sequence
angle
optional angular velocity
optional confidence/stability indicator
device state
\`\`\`

Do not tightly couple the protocol to Swift implementation details.

The protocol should be documented in the repository so both Rust and
Swift implementations can independently conform to it.

Prefer versioned protocol definitions once the protocol becomes
nontrivial.

------------------------------------------------------------------------

## 9. Apple Companion Application

### 9.1 Technology

The companion application should be written in:

-   Swift
-   SwiftUI

The app is intended primarily for Apple devices, initially likely
iPhone.

### 9.2 Responsibilities

The Swift application should:

-   Discover the Knife Sharpener device.
-   Establish/maintain a BLE connection.
-   Display the live sharpening angle prominently.
-   Indicate whether the measurement is stable/valid.
-   Display device connection/status information.
-   Guide the user through calibration.
-   Eventually provide device configuration.
-   Eventually provide additional sharpening-session UX.

The UI should remain simple during the prototype phase.

The primary screen should make the answer to the user's core question
immediately obvious:

> "What angle am I holding the knife at right now?"

### 9.3 Suggested Swift Architecture

A simple separation is preferred:

\`\`\` text
SwiftUI Views
      │
      ▼
Observable View Model
      │
      ▼
KnifeSharpenerService
      │
      ▼
BLE Transport
      │
      ▼
ESP32
\`\`\`

Keep CoreBluetooth-specific implementation behind a service/transport
abstraction.

Avoid embedding BLE calls directly throughout SwiftUI views.

### 9.4 Swift Data Model

The app should represent concepts such as:

\`\`\` text
KnifeSharpenerDevice
Measurement
AngleReading
CalibrationState
ConnectionState
DeviceConfiguration
\`\`\`

The exact model should evolve with the protocol.

------------------------------------------------------------------------

## 10. Repository Organization

A reasonable eventual repository layout is:

\`\`\` text
knife-sharpener/
├── AGENTS.md
├── README.md
├── docs/
│   ├── architecture.md
│   ├── protocol.md
│   ├── hardware.md
│   └── calibration.md
├── hardware/
│   ├── bom/
│   ├── schematics/
│   └── mechanical/
├── firmware/
│   └── ...
├── app/
│   └── ...
├── tools/
│   └── ...
└── tests/
    └── ...
\`\`\`

Do not force this exact structure if the actual toolchain benefits from
another organization, but preserve the conceptual separation.

------------------------------------------------------------------------

## 11. Development Phases

### Phase 0 --- Hardware Bring-Up

Goal: prove the ESP32 and IMU communicate.

Tasks:

-   Flash Rust firmware.
-   Establish USB/programming workflow.
-   Initialize IMU.
-   Read accelerometer.
-   Read gyroscope.
-   Log raw values.
-   Verify expected response when physically rotating the device.

Deliverable:

> Reliable raw IMU data on the ESP32 using Rust.

### Phase 1 --- Orientation

Goal: turn raw data into a useful orientation estimate.

Tasks:

-   Establish coordinate conventions.
-   Convert raw measurements to physical units.
-   Implement gravity-based orientation estimation.
-   Add gyroscope integration.
-   Add filtering/sensor fusion.
-   Measure latency and stability.

Deliverable:

> Stable real-time orientation estimate.

### Phase 2 --- Sharpening Angle

Goal: convert orientation into the actual user-facing knife angle.

Tasks:

-   Define sharpening-angle geometry.
-   Establish sensor-to-knife transform.
-   Add calibration offset.
-   Test against known physical angles.
-   Quantify error and repeatability.

Deliverable:

> Prototype can report knife angle with known, measured accuracy.

### Phase 3 --- BLE

Goal: transmit measurement data.

Tasks:

-   Implement BLE advertising.
-   Define service/characteristics.
-   Connect from a test client.
-   Stream angle data.
-   Add connection/device state.

Deliverable:

> External device can receive live angle measurements.

### Phase 4 --- SwiftUI App

Goal: create the first real user experience.

Tasks:

-   Create iOS SwiftUI app.
-   Implement BLE discovery.
-   Connect to ESP32.
-   Decode protocol.
-   Display live angle.
-   Display connection state.

Deliverable:

> Phone displays live sharpening angle from the physical prototype.

### Phase 5 --- Calibration and Refinement

Tasks:

-   Implement guided calibration.
-   Persist calibration.
-   Improve filtering.
-   Handle sensor movement.
-   Test across multiple knives.
-   Quantify accuracy.
-   Improve UI feedback.

Deliverable:

> Repeatable prototype suitable for realistic sharpening experiments.

### Phase 6 --- Productization Exploration

Only after the measurement concept works:

-   Custom PCB
-   Smaller MCU/module
-   Battery
-   Charging
-   Enclosure
-   Mechanical attachment
-   Manufacturing considerations
-   Power optimization
-   Robust firmware update process
-   More polished application

------------------------------------------------------------------------

## 12. Testing Strategy

Testing should exist at multiple levels.

### Rust unit tests

Pure mathematical code should be testable without hardware.

Examples:

-   Vector math
-   Coordinate transformations
-   Angle calculations
-   Calibration transformations
-   Filter behavior
-   Protocol serialization/deserialization

Prefer pure functions for mathematical transformations so they can be
tested on a normal host machine.

### Hardware tests

Test:

-   IMU initialization
-   Sensor reads
-   Sampling rate
-   BLE behavior
-   Calibration storage
-   Power behavior

### Physical validation

Use known physical reference angles to determine:

-   Absolute error
-   Repeatability
-   Drift
-   Response time
-   Noise
-   Behavior while the knife is moving

Do not declare the measurement system accurate solely because the
numbers "look right."

------------------------------------------------------------------------

## 13. Engineering Principles

### Prefer simple, explicit designs

This is a prototype and learning project. Favor understandable
architecture over abstraction for abstraction's sake.

### Separate concerns

Hardware drivers, measurement mathematics, communication, and UI should
be independently testable.

### Keep the protocol stable and documented

Rust and Swift are separate implementations. The protocol is the
contract between them.

### Make coordinate systems explicit

Document:

-   IMU X/Y/Z axes
-   Device axes
-   Knife axes
-   Sharpening-plane axes
-   Positive rotation directions
-   Angle zero/reference

Coordinate ambiguity is likely to cause more bugs than difficult
mathematics.

### Measure before optimizing

When deciding whether filtering, sampling rate, sensor choice, or
hardware changes are needed, collect data first.

### Prefer replaceable hardware components

The IMU should be replaceable without rewriting the measurement system.

### Avoid premature production complexity

Do not add:

-   Cloud services
-   User accounts
-   Authentication
-   Databases
-   Remote telemetry
-   Complex mobile architecture

unless a later requirement explicitly calls for them.

------------------------------------------------------------------------

## 14. Agent Guidance

Agents working on this repository should first read this file and then
inspect the actual repository state before making assumptions.

Important rules:

1.  **Do not assume hardware that is not present in the repository.**
2.  **Do not replace Rust firmware with C/C++ because an ESP32 example
    uses it.**
3.  **Do not assume a particular IMU until the BOM/schematic/code
    confirms it.**
4.  **Do not hard-code sensor orientation offsets when they belong in
    calibration.**
5.  **Do not bury BLE implementation throughout SwiftUI views.**
6.  **Do not introduce unnecessary cloud/backend infrastructure.**
7.  **Do not optimize sensor algorithms before establishing baseline
    measurements.**
8.  **Keep hardware-specific code separated from
    mathematical/algorithmic code.**
9.  **Document non-obvious coordinate transforms and units.**
10. **Use SI units internally where practical.**
11. **Preserve a clear distinction between raw sensor data, orientation,
    calibration, and final sharpening angle.**
12. **When changing the BLE protocol, update its documentation and both
    sides of the implementation.**
13. **Prefer host-testable pure Rust functions for mathematical logic.**
14. **When modifying hardware assumptions, update the relevant hardware
    documentation/BOM.**
15. **Favor incremental milestones that leave the prototype in a
    runnable state.**

------------------------------------------------------------------------

## 15. Repository and Project Boundaries

The project uses **one Git repository** containing multiple independently buildable components.

Recommended root structure:

\`\`\`text
knife-sharpener/
├── AGENTS.md
├── README.md
├── firmware/                 # ESP32 Rust project
│   ├── Cargo.toml
│   ├── Cargo.lock
│   └── src/
├── app/                      # Apple Swift/SwiftUI project
│   ├── KnifeSharpener.xcodeproj
│   └── KnifeSharpener/
├── protocol/                 # Shared BLE contract/documentation
│   ├── README.md
│   ├── ble.md
│   ├── characteristics.md
│   └── examples/
├── hardware/
│   ├── bom/
│   ├── schematics/
│   └── mechanical/
├── docs/
│   ├── architecture.md
│   ├── calibration.md
│   └── ...
└── tests/
\`\`\`

### Repository principle

Keep the ESP32 firmware and Apple application in the same repository because they are two parts of the same physical product and share a critical BLE interface.

However, they must remain **independently buildable projects**:

- \`firmware/\` is an independent Rust/Cargo project.
- \`app/\` is an independent Swift/Xcode project.
- Building one should not require the other.
- Avoid creating a monolithic cross-platform build system unless a future requirement justifies it.

This organization intentionally makes the software components easy to split into separate repositories later if the project grows into a larger product with independent teams or release processes.

### Shared protocol is a first-class component

The BLE protocol is the contract between the firmware and application and should not exist only implicitly in source code.

Document in \`protocol/\`:

- BLE service UUIDs
- Characteristic UUIDs
- Read/write/notify behavior
- Data types and units
- Serialization/encoding
- Device states
- Calibration messages
- Telemetry format
- Protocol/version compatibility
- Example messages/data

When changing the BLE protocol:

1. Update the protocol documentation.
2. Update the Rust firmware implementation.
3. Update the Swift application implementation.
4. Add/update tests or example data where practical.

The protocol should be designed so an agent working only on \`app/\` can understand the ESP32 interface without reverse-engineering the firmware.

### Independent agent ownership

It is expected that different agents may work on different components.

An agent working on the iOS application should treat:

\`\`\`text
protocol/
\`\`\`

as the authoritative interface contract and may inspect:

\`\`\`text
firmware/
\`\`\`

for reference implementation details.

An agent working on firmware should likewise treat \`protocol/\` as the interface contract and should not make undocumented changes that break the application.

The \`app/\` directory should remain sufficiently self-contained that an iOS-focused agent can develop the entire application without requiring the project owner to have Swift expertise.

---

## 16. Current Priority

The immediate objective is the **initial hardware prototype**.

The current development path is:

\`\`\` text
Purchase prototype BOM
        ↓
ESP32 + IMU hardware bring-up
        ↓
Rust firmware
        ↓
Reliable orientation
        ↓
Sharpening-angle calculation
        ↓
BLE telemetry
        ↓
Swift/SwiftUI companion app
        ↓
Calibration
        ↓
Accuracy / usability refinement
\`\`\`

The project should remain focused on proving the core proposition:

> A small, practical ESP32-based device can accurately measure the angle
> of a knife during sharpening and communicate that measurement to a
> user-facing Apple application in real time.
