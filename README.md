# Swerve Drive Robot — VEX Over Under (2024)

**A custom 4-wheel swerve drive robot** built by Marin Robotics team 2456V for the 2024 VEX Robotics ["Over Under"](https://www.youtube.com/watch?v=dvDqEI7qO34) competition season. Swerve drive is extremely rare in VEX — nearly all teams use standard tank or mecanum drives. We designed and programmed a drivetrain where all four wheels rotate independently, giving the robot full omnidirectional movement while maintaining the traction advantages of standard wheels. Here's a [clip of it competing in a match during a 2024 tournament](https://www.youtube.com/live/0YSf7jw0BNo?si=Z3bo6z15tt2kNRbB&t=9045) (timestamped at 2:30:45).


<img src="./swerve_drive_final.jpeg" alt="Robot photo" width="500"/>

## Design

- **4-wheel independent swerve:** Each module has a drive motor and an angle motor, allowing every wheel to point in any direction. The robot can translate, rotate, or do both simultaneously — all with full ground contact on every wheel.
- **Field-oriented control with GPS:** The driver can toggle between field-oriented and robot-oriented modes. In field-oriented mode, "up" on the stick always drives toward the same side of the field regardless of the robot's heading.
- **Team color mirroring:** GPS coordinates and heading are automatically flipped for the red alliance, so the same driver code and autonomous routines work from either side of the field without duplication.
- **Automated match loading:** The shooter uses limit switches to detect when a triball is loaded and when the mechanism is wound up, enabling a fully hands-off fire-and-reload cycle — the driver just toggles the mode on.
- **Autonomous selector:** Six pre-match selectable routines (red/blue × defense/offense/nothing) using the same swerve vector system as driver control.

## Implementation Details

### Swerve Vector Math

Each of the four swerve modules has two motors: a **drive motor** that spins the wheel and an **angle motor** that rotates the entire module to point the wheel in any direction. By computing independent target angles and speeds for all four modules simultaneously, the robot can translate in any direction, rotate in place, or do both at once.

The core of the drivetrain is vector addition. Each control input (translation from the left stick, rotation from the right stick) is decomposed into per-module vectors based on that module's position on the chassis. These vectors are summed and then normalized so that no module exceeds its maximum speed while preserving the intended direction ratios across all four wheels.

A key optimization: when a target angle is more than 90° from a module's current heading, the code flips the target by 180° and reverses the drive motor instead. This means no wheel ever needs to rotate more than 90° to reach any target direction, dramatically improving responsiveness.

### Proportional Control & Speed Scaling

Angle motors use a proportional controller to converge on their target heading. Drive motor speeds are scaled by `cos(largest_angle_error)` across all modules — this means the robot reduces its drive power when any wheel is still rotating toward its target angle, preventing jerky motion and wasted energy during direction changes. The robot only moves at full speed when all wheels are properly aligned.

### Field-Oriented Control

Using the VEX GPS sensor, the robot tracks its position and heading on the field in real time. Translation vectors from the joystick are rotated by the robot's current heading so that controls remain consistent relative to the field. The system also handles team color mirroring: on the red alliance, GPS coordinates and heading are automatically flipped so that the same control code and autonomous routines work from either side of the field.

### Automated Match Loading

The shooter mechanism uses two limit switches — `triball_loaded` to detect when a triball is placed in the shooter, and `shooter_ready` to detect when the mechanism is fully wound up. When toggled into match-load mode, the robot detects the triball, waits briefly for the operator's hand to clear, fires, and then automatically winds up for the next shot — no driver input needed beyond toggling the mode on.

### Autonomous

The robot has six selectable autonomous routines (red/blue × defense/offense/nothing), chosen pre-match via a physical button. Autonomous movement uses timed drive commands with the same swerve vector system used in driver control, combined with team-aware coordinate transforms so routines don't need to be written twice for each alliance.

## Platform

- **Runtime:** VEX V5 with [PROS](https://pros.cs.purdue.edu/) (open-source C++)
- **Drive motors:** 4× blue cartridge (600 RPM) for drive, 4× green cartridge (200 RPM) for steering
- **Sensors:** VEX GPS, 2× limit switches (shooter), 1× digital input (auton selector)
- **Module positions:** Front-left (45°), Front-right (315°), Back-left (135°), Back-right (225°)
