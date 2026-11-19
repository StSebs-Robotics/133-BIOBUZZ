# Field-Centric Mecanum Drive

Field-centric driving for the mecanum drivetrain. Pushing the left stick up always moves the robot away from the driver, whichever way the robot is facing. There are two TeleOps with the same controls:

| TeleOp (Driver Station name) | Uses | When |
|---|---|---|
| **Field Centric Drive** | [`MecanumDrive`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDrive.java) with the IMU for heading | Works now, needs only the 4 motors and the hub IMU |
| **Pedro Field Centric Drive** | Pedro Pathing `Follower` with odometry for heading and position | After the odometry pods are mounted and tuned (see [Pedro setup](../TeamCode/src/pedro/README.md)) |

## Controls (gamepad 1)

| Input | Action |
|---|---|
| Left stick | Drive (field-centric: up = away from the driver) |
| Right stick X | Turn |
| Right trigger | Slow mode, analog: full press caps power at 35% |
| Left bumper (hold) | Robot-centric: up = wherever the robot's front points |
| Options | Reset heading. Point the robot away from the driver first. The controller rumbles to confirm |

**At the start of a match** the robot must face away from the driver when the OpMode initialises, because that direction becomes field "forward". If it starts facing another way, either press Options once it's turned, or call `drive.setHeading(...)` in code (see [Heading](#heading)).

## Files

| File | What |
|---|---|
| [`opmodes/FieldCentricTeleOp.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/FieldCentricTeleOp.java) | Reads the sticks, shapes them, calls the drive. Shows loop time and battery |
| [`subsystems/MecanumDrive.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDrive.java) | Motors, IMU, field/robot-centric maths, optional closed-loop mode |
| [`subsystems/CachedMotor.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/CachedMotor.java) | Wraps a motor so `setPower` only talks to the hub when the value actually changed |
| [`opmodes/PedroFieldCentricTeleOp.java`](../TeamCode/src/pedro/java/org/firstinspires/ftc/teamcode/opmodes/PedroFieldCentricTeleOp.java) | Same controls through Pedro, tracks X/Y/heading |

## Robot configuration

Everything hardware-specific is a constant at the top of `MecanumDrive`. The Pedro constants read the same values, so you change them in one place.

| Constant | Default | Meaning |
|---|---|---|
| `FRONT_LEFT`, `FRONT_RIGHT`, `BACK_LEFT`, `BACK_RIGHT` | `front_left_drive`, … | Motor names. They must match the Driver Station config exactly |
| `IMU_NAME` | `imu` | Hub IMU name |
| `HUB_LOGO`, `HUB_USB` | `UP`, `FORWARD` | How the Control Hub is mounted. Wrong values make heading wrong when the robot turns |
| `STRAFE_MULTIPLIER` | `1.1` | Mecanum wheels strafe slower than they drive. Raise it (1.0–1.2) if strafing is noticeably slow |
| `MAX_TICKS_PER_SEC` | `2800` | Only for closed-loop mode, see below. Measure it |

Motor directions are set in the constructor: the left side is `REVERSE` and the right side is `FORWARD`. If a wheel spins backwards when you drive forward, flip that one motor's direction there.

### Checking a new robot

1. Prop the robot up so the wheels are off the ground.
2. Run **Field Centric Drive** and hold **left bumper** (robot-centric) so heading can't confuse the test.
3. Stick up: all four wheels should roll forward. If one doesn't, flip its direction.
4. Stick right: the robot should strafe right. Front-left and back-right spin forward, the other two backward. If it's the other way round, the motor names are swapped between front and back.
5. Right stick right: the robot should turn clockwise.
6. Release the bumper and turn the robot 90° by hand. Stick up should still drive away from you. If it drives sideways, or the wrong way as you turn, check `HUB_LOGO` / `HUB_USB`.

## How it works

### Field-centric maths

The IMU gives the robot's heading θ (counter-clockwise positive, 0 = field forward). The stick asks for a direction in field terms. `driveFieldCentric` rotates that request by −θ into the robot's own frame:

```
robotForward = forward·cos θ − strafe·sin θ
robotStrafe  = forward·sin θ + strafe·cos θ
```

`driveRobotCentric` then mixes the result into the four wheels:

```
FL = forward + strafe + turn      FR = forward − strafe − turn
BL = forward − strafe + turn      BR = forward + strafe − turn
```

If any wheel would exceed 1, all four are scaled down together. That keeps the direction of travel when you drive and turn at full stick. Then everything is multiplied by the max-power cap (slow mode).

### Stick shaping

- **Deadband 0.05**: ignores stick drift.
- **Squared response**: half stick gives about quarter power, for fine control near the centre and full power at the edge.
- Translation is shaped as one vector, not per axis, so diagonal moves stay at the angle you push.
- Turning is scaled by `TURN_SCALE = 0.8`.

These constants are at the top of `FieldCentricTeleOp`, and the Pedro TeleOp reuses them.

### Heading

| Method | Use |
|---|---|
| `resetHeading()` | The robot's current facing becomes field forward (the Options button) |
| `setHeading(radians)` | Tell the drive the robot's current field heading, e.g. `setHeading(Math.toRadians(90))` if it starts facing the driver's left |
| `getHeading()` | Current heading, radians, CCW positive, in (−π, π] |

These apply an offset to the IMU reading; the IMU itself isn't reset. The Pedro TeleOp does the same, so pressing Options never corrupts the odometry heading that autonomous paths use.

## Loop speed

The drive is built so that a loop costs as few hub round trips as possible. Each round trip is about 1–3 ms over the hub's serial link, and blocks the loop.

| Technique | Where | Saves |
|---|---|---|
| Bulk reads in `MANUAL` mode, cleared once in `drive.update()` | `MecanumDrive` constructor / `update()` | One transaction per hub per loop for every encoder and digital read |
| IMU read once per loop, cached | `update()` | The IMU is I²C, the most expensive read in the loop |
| Battery voltage polled every 500 ms | `update()` | One transaction most loops |
| `CachedMotor` skips writes that changed less than 0.5% | every `setPower` | Up to 4 transactions per loop. Stick noise would otherwise resend all four motors every loop |

**Rule:** call `drive.update()` exactly once, at the top of every loop, before reading anything. The TeleOp shows the loop time on telemetry. Watch it when you add mechanisms.

If you add hardware read on the main thread, it uses the same bulk cache for free. The [launcher](launcher.md) runs on its own thread and does its own bulk read on its hub, so it doesn't interfere with this.

### `CachedMotor` options

`drive.motorParams` is shared by all four drive motors. Set these in the `MecanumDrive` constructor, or change them at runtime:

| Field | Default | Effect |
|---|---|---|
| `writeThreshold` | `0.005` | Smallest power change worth sending. 0 and ±1 are always sent |
| `slewRatePerSec` | `0` (off) | Max power change per second. Try `6.0` to soften jerky starts. `stop()` ignores it |
| `kStatic` | `0` | Minimum power for any non-zero command. Try `0.05` if the robot won't creep at low stick |
| `voltageCompensation` | `false` | Scale power by 12 V ÷ battery so speed doesn't fade as the battery drains |
| `velocityWriteThreshold` | `5` ticks/s | The same skipping idea, for closed-loop mode |

## Optional: closed-loop drive

```java
drive.setClosedLoop(true);
```

In closed-loop mode each wheel gets a **velocity target** instead of a power, and the hub's built-in velocity PIDF holds it. That loop runs on the hub, so it costs no extra traffic per loop. You get the same speed at any battery level and straighter driving. It requires:

1. Encoders cabled on all four drive motors.
2. `MAX_TICKS_PER_SEC` measured: run a wheel at full power in `RUN_USING_ENCODER` and read `getVelocity()`. As a rough guide, goBILDA 312 rpm ≈ 2800, 435 rpm ≈ 3900, 223 rpm ≈ 2000. Too high and full stick saturates. Too low and the robot never reaches top speed.
3. The default PIDF (`applyDefaultVelocityPIDF`: F = 32767 / MAX_TICKS_PER_SEC, P = 0.1·F) is a starting point. Use `setVelocityPIDF(p, i, d, f)` to hand-tune it.

It's off by default, and the TeleOp doesn't turn it on.

## Pedro version

**Pedro Field Centric Drive** has the same controls, but drives through Pedro's `Follower`. That means the robot's X/Y position is tracked the whole time, ready for auto-aim, drive-to-point or carrying the pose over from autonomous. Pedro's convention is +strafe = left and +turn = counter-clockwise; the TeleOp converts. Setup and tuning order are in [`TeamCode/src/pedro/README.md`](../TeamCode/src/pedro/README.md). Once Pedro is tuned, use only one of the two TeleOps in matches, so only one thing ever commands the drive motors.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Stick up drives sideways or backwards from the start | The robot wasn't facing away from the driver at init. Press Options |
| Direction drifts more the more you turn | `HUB_LOGO` / `HUB_USB` don't match how the hub is mounted |
| One wheel fights the others | That motor's direction is wrong |
| Strafing curves | Weight imbalance or a slipping roller. A slightly different `STRAFE_MULTIPLIER` can help if strafe is just slow |
| Robot won't move at small stick | Static friction. Set `motorParams.kStatic ≈ 0.05` |
| Loop time grows as mechanisms are added | Something reads hardware outside the bulk cache (I²C sensors, `getVoltage()`), or `update()` is called more than once per loop |
