# Pedro Pathing + Panels

Everything under `src/pedro/java` is compiled only when `usePedro = true` in
`TeamCode/build.gradle`, which is the current setting. Set it to `false` to build without Pedro
or Panels.

Installed (latest as of 2026-11-19, APK verified to build):

| Library | Version | Use |
|---|---|---|
| `com.pedropathing:revhub` (+ `core`) | 3.0.2 | Drivetrain, localizers, Follower |
| `com.pedropathing:tuning` | 1.0.3 | Tuning web UI, `http://192.168.43.1:10158` |
| `com.pedropathing:ivy` | 1.0.0 | Command scheduler |
| `com.pedropathing:telemetry` | 1.0.0 | Selectable on-robot menus |
| `com.bylazar:fullpanels` | 1.0.20 | Panels dashboard, `http://192.168.43.1:8001` |

Don't add `com.pedropathing:ftc`. It's the old Pedro 2 library and conflicts with `core` 3.

## What's here

| File | Purpose |
|---|---|
| `pedro/Constants.java` | Drivetrain, 3-wheel + IMU localizer, Foresight config. `createFollower()` |
| `pedro/Tuning.java` | Registers tuners with Pedro's tuning web UI |
| `pedro/procedures/*` | Tuners copied unmodified from the Pedro Quickstart |
| `opmodes/PedroFieldCentricTeleOp.java` | Same controls as `FieldCentricTeleOp`, but tracks pose |

Motor names, directions and hub orientation come from `subsystems/MecanumDrive.java`, so change
them there once and both drive stacks follow.

## Setup (after the pods are mounted)

1. Wire each pod to a motor encoder port. Note which motor's port each pod uses.
2. In `Constants.localizerConfig`, set `leftEncoderName` / `rightEncoderName` /
   `strafeEncoderName` to those motor names.
3. Deploy.
4. Run the tuners from the tuning web UI in order, pasting each printed code block into `Constants`:
   1. **Mecanum Tuner**: motor directions
   2. **Three Wheel IMU Tuner**: pod offsets, ticks-to-inches, encoder directions
   3. **Tests → Localization**: drive around and check X/Y/heading look right
   4. **Foresight Tuner**: path-following constants (`foresightConfig`)
   5. **Tests → Line / Curve / Hold**
5. Drive with **Pedro Field Centric Drive**. Once happy, retire `Field Centric Drive` and
   `MecanumDrive`'s motor code, so only one thing ever commands the drive motors.

Until step 4.4 is done `foresightConfig` is `null`: TeleOp and localization work, while
`follow()` / `hold()` (autonomous paths) do not.

## Updating Pedro later

Bump both versions in `TeamCode/build.gradle` and re-copy `procedures/*` from the Quickstart
at the matching commit. The tuners are tied to the library version.
