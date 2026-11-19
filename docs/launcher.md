# Launcher (MPC Flywheel Control)

Holds the launcher flywheel at a target speed using model-predictive control (MPC). It runs on its own thread at 100 Hz, so speed control doesn't depend on how fast the OpMode loop is. It recovers quickly after each ball, and tells you when it's ready to shoot.

```java
LauncherMPC launcher = new LauncherMPC(this);   // in init: thread starts, motor stays off
launcher.setTargetVelocity(1800);               // ticks/s. No update() call needed
if (launcher.isReady()) feed();                 // at speed and settled
launcher.off();                                 // coast
```

The thread stops and zeroes the motors on its own within ~50 ms of the OpMode ending. Calling `launcher.close()` in `stop()` as well is still good practice; see [Not yet verified on the robot](#not-yet-verified-on-the-robot).

**Contents:** [Setting up a new launcher](#setting-up-a-new-launcher) · [Tuning recipe](#tuning-recipe) · [Using it in OpModes](#using-it-in-opmodes) · [How it works](#how-it-works) · [Parameter reference](#parameter-reference) · [Tools](#tools) · [Troubleshooting](#troubleshooting) · [Changes from last year](#changes-from-last-years-launcher)

## Setting up a new launcher

1. **Hardware config.** Edit `LauncherMPC.defaultConfig()` in [`LauncherMPC.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherMPC.java):

   ```java
   return new Config()
           .motor("launcher", DcMotorSimple.Direction.FORWARD)
           .motor("launcher2", DcMotorSimple.Direction.REVERSE)   // second flywheel motor, if any
           .encoder(0)                                            // which motor has the encoder cable
           .name("launcher");                                     // params file name
   ```

   Directions must make **positive power spin the wheel the shooting way**. The encoder must also read positive that way. SysId tells you if it doesn't.

2. **Run Launcher SysId**, then **Launcher MPC Tuner** (both in the `tuning` group on the Driver Station). See the next section.

Tuned values are saved on the Control Hub in `/sdcard/FIRST/launcher/launcher.properties`, and every OpMode that creates a `LauncherMPC` loads them at init. Retuning needs no code changes or rebuild. Without a file, the defaults in [`LauncherParams.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherParams.java) are used.

## Tuning recipe

About 15 minutes for a new launcher.

### Step 1: Launcher SysId (measure the launcher)

Remove balls, keep hands clear, and start with a battery at 12.5 V or more. Press start. It runs open-loop power steps (0.30 → 0.60 → 0.45 → 0.90 → 0.70 → 1.00 → 0.50 → 0.80 → 0.35 → coast). Each step is held until the speed settles, for about 30 s in total, and it then fits the [model](#the-model).

| Result | What it means | Good when |
|---|---|---|
| `tau` | Seconds to reach 63% of a speed change | |
| `kV` | Ticks/s per unit of power at 12 V | |
| `kS` | Friction, in ticks/s | Positive |
| `delay` | Time between `setPower` and the encoder seeing it | Under ~40 ms |
| `fit error` / r² | How well the model matches | r² > 0.99 |
| steady-state check | `kV` / `kS` from the settled speeds only | `kV` within 10% of the fit |
| Top speed @12V | Physical limit | Keep match targets at or below 85% of this |

The 85% rule matters. Without that headroom there's nothing left to recover from a shot or a sagging battery.

Press **A** to save. This writes the model, and also sets `horizonSec ≈ tau` and `measNoise` from the measured noise. MPC weights already in the file are kept. Press **X** to write the raw log as CSV.

Re-run SysId whenever the mechanism changes: wheel, gearing, compression, belt, added mass.

### Step 2: Launcher MPC Tuner (tune live)

| Button | Action |
|---|---|
| dpad up/down | Select parameter |
| dpad right/left | Increase / decrease it (applies immediately). Hold **left bumper** for ×10 steps |
| A | Step test: toggle between LOW and HIGH target |
| B | Spin up to HIGH / off |
| X | **Auto test**: off → HIGH → LOW → HIGH → off, then prints `AUTO SCORE` |
| Y | Save to the Control Hub file |
| back / share | Write the last 60 s as CSV |

Every step and every shot is measured automatically, and the last four are shown:

```
STEP 970→1624  rise 348ms  OS 3.2%  settle 474ms  ss 8.5  IAE 0.234
DIP  -154 t/s  recover 166ms  then OS 0.6%
```

**rise** is 10→90% time. **OS** is overshoot. **settle** is the time until it stays within the ready band. **ss** is the RMS error once settled. **IAE** is the total error, normalised. **DIP** is a shot: how far the speed fell, how long it took to recover, and any over-speed afterwards (that sends the next ball long).

**Method:** set `targetHigh` / `targetLow` to your real shooting speeds. Press **X**, and **feed 3 balls during the second HIGH hold**. Then change one parameter, run X again, and keep whatever lowers `AUTO SCORE`. Work in this order:

1. **`rDu`**: speed against smoothness. Lower it until spin-up stops getting faster, or until `chatter` passes ~0.008/loop (the motor starts to buzz).
2. **`loadNoise`**: shot recovery. Raise it if `recover` is slow. Lower it if the speed wanders while holding.
3. **`measNoise`**: raise it if power is noisy while holding speed.
4. **`shotDetectSigma`**: watch `shots detected`. Lower it if shots are missed. Raise it if it counts shots with no balls fed.
5. `horizonSec`, `qTerminal`: rarely matter for a flywheel.

Press **Y** to save when you're happy.

## Using it in OpModes

```java
public class MyTeleOp extends LinearOpMode {
    @Override public void runOpMode() {
        LauncherMPC launcher = new LauncherMPC(this);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.aWasPressed()) launcher.setTargetVelocity(1600);
            if (gamepad2.bWasPressed()) launcher.off();
            if (gamepad2.right_bumper && launcher.isReady()) feeder.push();
            telemetry.addData("launcher", "%.0f / %.0f %s", launcher.getVelocity(),
                    launcher.getTarget(), launcher.isReady() ? "READY" : "");
            telemetry.update();
        }
        launcher.close();
    }
}
```

In autonomous, wait for `isReady()` before each feed, with a timeout in case a ball jams.

### API

| Method | Notes |
|---|---|
| `new LauncherMPC(opMode)` / `(opMode, config)` | Sets up the motors, loads params, starts the thread. Motor stays off |
| `setTargetVelocity(ticksPerSec)` | ≤ 0 means coast |
| `setTargetRpm(rpm)` | Flywheel RPM, using `ticksPerRev` |
| `off()` | Coast (power 0, `FLOAT`) |
| `close()` | Stop the thread and the motors. Safe to call twice |
| `isReady()` | Within `max(readyTolFrac·target, readyTolMin)` of the target for `readySettleMs` |
| `getVelocity()`, `getTarget()`, `getMode()` | Latest values |
| `getShotCount()` | Balls detected from speed dips |
| `getState()` | Full snapshot: target, velocity, estimate, load, power, battery, loop/solve time, ready, shot flags |
| `getError()` | The exception that killed the control thread, or `null` |
| `getParams()` / `setParams(p)` / `saveParams(comment)` | Read, change and persist tuning at runtime |
| `startLogging(n)`, `getLog()`, `getLogSince(t)`, `dumpLog(prefix)` | Per-loop ring buffer and CSV export |
| `setOpenLoopPower(p)` | Raw power, no control (SysId uses it) |

Every call is safe from any thread. Setters only store a value, and getters return the latest snapshot the control thread published, so nothing blocks the OpMode loop.

## How it works

```
 every 10 ms on the LauncherMPC thread
 ┌──────────────┐   ┌───────────────────┐   ┌───────────────┐   ┌──────────────┐   ┌────────────┐
 │ bulk read    │──▶│ Kalman filter     │──▶│ delay         │──▶│ MPC: plan    │──▶│ setPower   │
 │ (own hub)    │   │ speed + load,     │   │ compensation  │   │ 15 steps,    │   │ (skipped if│
 │ + battery@10Hz│  │ shot detection    │   │               │   │ send step 1  │   │ unchanged) │
 └──────────────┘   └───────────────────┘   └───────────────┘   └──────────────┘   └────────────┘
```

### The model

```
tau · dv/dt = −v + kV · power · (battery / 12 V) − kS − load
```

The flywheel is a first-order system. The battery term means the controller already knows that 0.7 power at 11.5 V is weaker than at 13 V. **load** is everything the model gets wrong: friction changing as it warms up, a slightly wrong `kV`, a ball rubbing. It is estimated continuously, not tuned.

### Kalman filter (speed + load)

Each loop, the filter compares the measured speed with what the model predicted, and splits the difference between "the speed reading is noisy" and "the load changed". The `measNoise`, `velNoise` and `loadNoise` parameters set that split. Because load is estimated, the speed settles exactly on target without an integral term to wind up.

### Shot detection

A ball going through is a sudden speed loss lasting ~30 ms, not a lasting load. Without special handling, the filter would read the drop as extra load. It would then keep pushing extra power after the speed had recovered, and the wheel would over-speed for about a second (2–4% in simulation), sending the next ball long.

So when speed falls faster than the model allows (a CUSUM test on the filter's prediction errors, threshold `shotDetectSigma`), the drop is treated as a shot. The speed estimate jumps to the measurement, and the load estimate is put back to its value from before the drop. A shot ends when the speed stops falling, or after 0.2 s at most. Detection is only switched on while holding near the target, because model error during spin-up would look like a shot. In simulation this brings the after-shot over-speed down to under ~1%. Detected shots are counted (`getShotCount()`).

### Delay compensation

There are a few ms between `setPower` and the encoder showing the effect: hub latency plus the hub's velocity filtering. SysId measures this `delay`. The controller rolls its estimate forward through the commands already sent but not yet visible, so it plans from where the wheel *will* be when the new command lands.

### MPC

Each loop it looks `horizonSec` ahead in `horizonSteps` steps and picks the power sequence that minimises

```
Σ q·((target − v_j)/kV)²   +   rDu · Σ (u_j − u_{j−1})²      with uMin ≤ u_j ≤ uMax
```

That is, it trades speed error against changes in power, within the motor's limits. Dividing by `kV` makes the weights unitless, so they carry over between launchers. The problem is a small quadratic program, solved with accelerated projected gradient (FISTA) and warm-started from the previous plan. Only the first power is sent, and it re-plans next loop. The solve is ~40 µs on a laptop; the Control Hub is slower, and the tuner shows the real `solve` time.

Full power is used when the model says it's needed. Spin-up from rest is limited by the motor, not the controller.

### Threading and hub access

- Velocity comes from `LynxModule.getBulkData()` on the launcher's own hub, a fresh read every loop. That call also refreshes the hub's bulk cache. Main-thread code that uses `MANUAL` caching (like [`MecanumDrive`](field-centric-drive.md#loop-speed)) keeps working and just sees slightly newer data.
- Per loop: one bulk read, plus one `setPower` per motor (skipped when the change is under 0.2%, via `CachedMotor`), plus a battery read 10 times a second.
- Safety: every 50 ms the thread checks that its OpMode is still the active one. Once it isn't, it zeroes the motors and exits. On an unexpected exception it also zeroes the motors and exits; `getError()` reports what happened.

## Parameter reference

All are in [`LauncherParams.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherParams.java) and saved to the `.properties` file. Out-of-range values are clamped.

| Parameter | Default | Set by | Meaning |
|---|---|---|---|
| `tau` | 0.60 s | SysId | Time constant |
| `kV` | 2400 | SysId | Ticks/s per unit power at `nominalVoltage` |
| `kS` | 80 | SysId | Friction offset, ticks/s |
| `delaySec` | 0.010 | SysId | Dead time |
| `nominalVoltage` | 12.0 | | Voltage the model is normalised to |
| `periodMs` | 10 | | Control period (100 Hz) |
| `horizonSec` | 0.60 | SysId (≈ tau) | Look-ahead |
| `horizonSteps` | 15 | | QP size |
| `qVel` | 1.0 | | Speed error weight. Leave it at 1 |
| `qTerminal` | 5.0 | Tuner | Extra weight at the end of the horizon |
| `rDu` | 0.05 | Tuner | Power-change weight: higher = smoother and slower |
| `uMin` / `uMax` | 0 / 1 | | Power limits. A negative `uMin` lets it brake when slowing down |
| `maxDuPerSec` | 0 (off) | | Extra slew limit on power |
| `maxIters` | 40 | | Solver iterations per loop |
| `measNoise` | 30 | SysId, Tuner | Encoder noise, ticks/s. Higher = smoother, slower to react |
| `velNoise` | 6 | | Unmodelled speed change per loop |
| `loadNoise` | 4 | Tuner | Load change per loop. Higher = faster shot recovery |
| `shotDetectSigma` | 3 | Tuner | Shot detector threshold, 0 = off |
| `readyTolFrac` / `readyTolMin` | 0.02 / 20 | | Ready band: `max(2% of target, 20 ticks/s)` |
| `readySettleMs` | 80 | | Time in the band before `isReady()` |
| `ticksPerRev` | 28 | | For `setTargetRpm` and RPM display only |

Defaults come from simulator sweeps (`sim.sh sweep`). After SysId, the tuner on the real launcher has the final say.

## Tools

### Desktop simulator

[`tools/launcher/sim.sh`](../tools/launcher/sim.sh) compiles the real controller classes (`MpcCore`, `SysIdFit`, `StepAnalyzer`, `TuneScore` are plain Java with no FTC imports) and runs them against a simulated flywheel. The simulation has encoder noise, dead time, timing jitter, battery sag, model error and 30 ms ball contacts.

```bash
./tools/launcher/sim.sh run                                    # standard scenario: spin-up, step down, step up, 3 shots
./tools/launcher/sim.sh run launcher.properties --csv run.csv  # your robot's params, write a CSV
./tools/launcher/sim.sh sweep launcher.properties              # best rDu / noise settings, robust to ±10% model error
./tools/launcher/sim.sh sysid --mismatch 0.2                   # check SysId recovers a known launcher
```

Options: `--mismatch X` (real launcher off from the model by ±X), `--shots N`, `--shotDrop X` (speed lost per ball, default 0.12), `--target T`, `--noise S`, `--seed N`. Use `adb pull /sdcard/FIRST/launcher/launcher.properties` to get the robot's params.

The sim prints the same metrics and `score` as the tuner (same code), so desk and robot numbers compare directly.

### CSV plotter

Open [`tools/launcher/plot.html`](../tools/launcher/plot.html) in a browser and drop in a CSV. It plots target, measured speed and the Kalman estimate, with power below. Scroll to zoom, drag to pan, double-click to reset. To get robot logs:

```bash
adb pull /sdcard/FIRST/launcher/ launcher-logs
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| SysId: "flywheel never moved forward" | Motor names wrong, the encoder isn't on the `encoder(i)` motor, or the direction makes positive power read negative |
| SysId r² < 0.98 | Something rubbing or slipping, a loose belt, or a battery too low to hold voltage |
| Settles below target and never reaches it | The target is above what the motor can do at this battery. Check the top speed from SysId |
| Motor buzzes while holding speed | `rDu` too low or `measNoise` too low |
| Slow recovery after shots | Raise `loadNoise`. Check the target isn't too close to top speed |
| Next ball goes long after a shot | Shots not being detected (watch `shots detected`). Lower `shotDetectSigma` |
| `shots detected` climbs with no balls | Raise `shotDetectSigma` or `measNoise` |
| Tuner shows `CONTROL THREAD DIED` | See the message. Usually a hardware name missing from the config |
| `Direct bulk read: NO (fallback)` in SysId | The motor's hub couldn't be found. Speed falls back to `getVelocity()`, which may be stale if the main thread uses `MANUAL` caching |

## Changes from last year's launcher

| DECODE (2025-26) | Now |
|---|---|
| Brute-force search over 9 power sequences (±0.05 steps, 2-step horizon) | Real MPC: 15-step horizon, constrained QP |
| Ran inside the OpMode loop, so its rate depended on everything else | Own thread at a fixed 100 Hz with its own bulk read |
| `kI` integral added after the MPC, with a 2% gate | Kalman load estimate. No integral to tune or wind up |
| No battery compensation | Battery voltage is part of the model |
| Model fitted by hand from telemetry (`kS` came out negative) | `Launcher SysId` fits it automatically, unbiased by encoder noise |
| Shots: integral reset, slow recovery | Shot detection, no over-speed after recovery, shot counting |
| Tune by editing `TeleConstants` and rebuilding | Tune live from the gamepad, saved on the Control Hub |

## Not yet verified on the robot

Everything above is checked in the desktop simulator and compiles against SDK 12. These have not been tested on hardware yet:

- **Solve time on the Control Hub.** It's shown in the tuner as `solve`. If it's over ~2 ms, lower `horizonSteps` or `maxIters`.
- **Self-stop when the OpMode ends.** The thread checks for this, but it hasn't been confirmed on a hub. Until it has, call `launcher.close()` in `stop()` / at the end of `runOpMode()`.
