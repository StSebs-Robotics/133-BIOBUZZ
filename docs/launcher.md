# Launcher (MPC Flywheel Control)

Holds the launcher flywheel at a target speed using model-predictive control (MPC). It runs on its own thread at 100 Hz, so speed control doesn't depend on how fast the OpMode loop is. It compensates for the battery, recovers quickly after each ball, and tells you when it's ready to shoot.

```java
LauncherMPC launcher = new LauncherMPC(this);   // in init: thread starts, motor stays off
launcher.setTargetVelocity(1800);               // ticks/s. No update() call needed
if (launcher.isReady()) feed();                 // at speed and settled
launcher.off();                                 // coast
```

This page covers setting it up, tuning it and using it. How it works inside (the maths, the design decisions and the evidence for them) is in **[launcher-internals.md](launcher-internals.md)**.

**Contents:** [How it works, in short](#how-it-works-in-short) · [Setting up a new launcher](#setting-up-a-new-launcher) · [Tuning](#tuning) · [Using it in OpModes](#using-it-in-opmodes) · [Shooting: recovery and `notifyShot()`](#shooting-recovery-and-notifyshot) · [Parameter reference](#parameter-reference) · [Tools](#tools) · [Troubleshooting](#troubleshooting) · [Changes from last year](#changes-from-last-years-launcher) · [Not yet verified on the robot](#not-yet-verified-on-the-robot)

## How it works, in short

Every 10 ms, on its own thread, the launcher:

1. **Reads the wheel speed** from the encoder (its own bulk read) and the **battery voltage** (10 times a second).
2. **Estimates the true speed and the "load"** with a Kalman filter. The load is everything the model gets wrong: friction, warm-up, a slightly wrong `kV`. Because it's estimated continuously, the speed settles exactly on target without an integral term.
3. **Detects balls going through** from the shape of the speed drop, and counts them.
4. **Plans the next 0.6 s of motor power** (the MPC). It uses a model of the flywheel to try power plans, and picks the one that best trades speed error against power changes, within the motor's 0–1 limits.
5. **Sends the first power of that plan** and plans again next loop.

The model is `tau · dv/dt = −v + kV · power · (battery / 12 V) − kS − load`. The battery term means the controller already knows that 0.7 power at 11.5 V is weaker than at 13 V, so performance doesn't fade as the battery drains.

## Setting up a new launcher

1. **Hardware config.** Edit `LauncherMPC.defaultConfig()` in [`LauncherMPC.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherMPC.java):

   ```java
   return new Config()
           .motor("motor3", DcMotorSimple.Direction.FORWARD)      // current launcher motor
           .motor("motor6", DcMotorSimple.Direction.REVERSE)      // e.g. a second flywheel motor, if any
           .encoder(0)                                            // which motor has the encoder cable
           .name("launcher");                                     // params file name
   ```

   Directions must make **positive power spin the wheel the shooting way**. The encoder must also read positive that way. SysId tells you if it doesn't.

2. **Tune** (next section): Launcher SysId, then Launcher Auto-Tune. Both are in the `tuning` group on the Driver Station.

Tuned values are saved on the Control Hub in `/sdcard/FIRST/launcher/launcher.properties`, and every OpMode that creates a `LauncherMPC` loads them at init. Retuning needs no code changes or rebuild. Without a file, the defaults in [`LauncherParams.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherParams.java) are used. You can copy the file between robots, or back it up, with `adb pull` / `adb push`.

## Tuning

About 10 minutes for a new launcher, mostly waiting:

| Step | OpMode | Time | What it sets |
|---|---|---|---|
| 1 | **Launcher SysId** | ~30 s | The model: `tau`, `kV`, `kS`, `delaySec`. Also starts `horizonSec` and `measNoise` |
| 2 | **Launcher Auto-Tune** | up to ~8 min | The weights: `rDu`, `loadNoise`, `measNoise` |
| 3 | Launcher MPC Tuner (optional) | as long as you like | Anything, by hand, with real balls |
| 4 | Your TeleOp with `notifyShot()` (optional) | 2 balls | The feeder timing, learned automatically |

Re-run step 1 (and then 2) whenever the mechanism changes: wheel, gearing, compression, belt, added mass. Keep the battery at 12.5 V or more while tuning.

### Step 1: Launcher SysId (measure the launcher)

Remove balls, keep hands clear, and press start. It runs open-loop power steps (0.30 → 0.60 → 0.45 → 0.90 → 0.70 → 1.00 → 0.50 → 0.80 → 0.35 → coast). Each step is held until the speed settles (at most 5 s), and it then fits the model to all of the data.

| Result | What it means | Good when |
|---|---|---|
| `tau` | Seconds to reach 63% of a speed change. Heavier wheels have bigger `tau` | |
| `kV` | Steady speed per unit of power at 12 V, ticks/s | |
| `kS` | Friction, in ticks/s. The wheel only starts turning above power `kS / kV` | Positive |
| `delay` | Time between `setPower` and the encoder showing it (hub latency plus the hub's velocity filtering) | Under ~40 ms |
| `fit error` / r² | How well the model matches the data | r² > 0.99 |
| steady-state check | `kV` / `kS` from the settled speeds only, as a cross-check | `kV` within 10% of the fit |
| Top speed @12V | `kV − kS`: the physical limit | Keep match targets at or below 85% of this |

**The 85% rule matters more than any tuning.** Recovering from a ball needs spare motor power. At 85% of top speed there's roughly 1.5 V of battery sag headroom left.

Press **A** to save. This writes the model, and also sets `horizonSec ≈ tau` and `measNoise` from the measured noise, without touching the other values in the file. **X** writes the raw log as CSV.

If the fit fails or r² is low, see [Troubleshooting](#troubleshooting).

### Step 2: Launcher Auto-Tune (automatic)

Press start and leave it. **No balls needed.** It finds `rDu`, `loadNoise` and `measNoise` by running the same test over and over with different values, then saves the best.

**Before start (init):**

| Button | Action |
|---|---|
| dpad up/down | HIGH speed. LOW is 85% of it. Set it near your real shooting speed |
| dpad left/right | Trial budget (default 30, about 8 min at most) |

**Each trial (~13 s):**
- LOW → HIGH → LOW → HIGH → LOW, with ~3 s holds. That's a game-like step size; big steps run at full power the whole way and say nothing about the tuning.
- In every hold, a **virtual shot**: the motor is cut for 0.1 × `tau` (about 60 ms) while the controller is told the power went through. To the controller that's an unexplained ~10% speed loss, the same thing a ball does.
- The trial score (lower is better) adds up step error over time (IAE), shot recovery time, over-speed after shots, and power chatter (motor buzz).

**The search:**
1. One warm-up trial (not scored), then a baseline with your current values.
2. For each of the three parameters: re-measure the current best (the launcher drifts as it warms up and the battery drains), then try it doubled and halved. Keep going in whichever direction keeps improving.
3. Repeat passes until a pass changes nothing, at most 3 passes.
4. A change only counts if it's at least 3% better than the average of the current best's recent scores. A close call is tested a second time and judged on its average.
5. **The final check:** the winner and your starting values are re-tested alternately, twice each. **It only saves if the winner is still better on average.** Otherwise it says your current values were already best and changes nothing.

**When it finishes** it shows before/after scores and values, and the last trials. The old file is kept as `launcher.properties.bak`, and **X** puts it back (undo). **B** at any time aborts without saving.

**What to expect:** it's cautious by design. In simulation it reliably fixes noise and chatter problems (for example `measNoise` set far too low, where it improved the real-shot score by 24%) and leaves good values alone. It won't move far from where it starts when the measurements are too noisy to be sure, which is why SysId goes first.

### Step 3 (optional): Launcher MPC Tuner (by hand)

Use it to check the result with real balls, or to adjust what Auto-Tune doesn't search (`shotDetectSigma`, `shotFeedforward`, `qTerminal`, `horizonSec`). Every change applies instantly.

| Button | Action |
|---|---|
| dpad up/down | Select parameter |
| dpad right/left | Increase / decrease it. Hold **left bumper** for ×10 steps |
| A | Step test: toggle between LOW and HIGH target |
| B | Spin up to HIGH / off |
| X | **Auto test**: off → HIGH → LOW → HIGH → off, then prints `AUTO SCORE` (lower is better) |
| right bumper | `notifyShot()`: press as you push a ball in |
| Y | Save to the Control Hub file |
| back / share | Write the last 60 s as CSV |

Every step and every shot is measured automatically, and the last four are shown:

```
STEP 970→1624  rise 348ms  OS 3.2%  settle 474ms  ss 8.5  IAE 0.234
DIP  -154 t/s  recover 166ms  then OS 0.6%
```

| Metric | Meaning |
|---|---|
| **rise** | 10→90% time of the step |
| **OS** | Overshoot past the target, % of the step size |
| **settle** | Time until it stays within the ready band |
| **ss** | RMS error at the end of the step (≈ encoder noise when settled) |
| **IAE** | Total error over the step, normalised by its size. The best single number |
| **DIP** | A shot: how far the speed fell, how long it took to get back in the band, and any over-speed afterwards (that sends the next ball long) |
| **chatter** | RMS power change per loop while holding speed. Above ~0.008 the motor starts to buzz |

**Method:** set `targetHigh` / `targetLow` to your real shooting speeds. Press **X**, and **feed 3 balls during the second HIGH hold**. Change one parameter, run X again, and keep whatever lowers `AUTO SCORE`. See the [parameter reference](#parameter-reference) for what each one does and its symptoms.

## Using it in OpModes

```java
public class MyTeleOp extends LinearOpMode {
    @Override public void runOpMode() {
        LauncherMPC launcher = new LauncherMPC(this);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.aWasPressed()) launcher.setTargetVelocity(1600);
            if (gamepad2.bWasPressed()) launcher.off();
            if (gamepad2.right_bumper && launcher.isReady()) {
                feeder.push();
                launcher.notifyShot();          // optional, see below
            }
            telemetry.addData("launcher", "%.0f / %.0f %s", launcher.getVelocity(),
                    launcher.getTarget(), launcher.isReady() ? "READY" : "");
            telemetry.update();
        }
        launcher.close();
    }
}
```

- **No `update()` call.** The thread does everything. Setters only store a value and getters return the latest snapshot, so nothing blocks your loop.
- **Never `sleep()` waiting for the launcher.** Check `isReady()` each loop instead.
- **Call `close()`** at the end (or in `stop()`). The thread also stops itself when the OpMode ends, but that hasn't been verified on a hub yet.

### API

| Method | Notes |
|---|---|
| `new LauncherMPC(opMode)` / `(opMode, config)` | Sets up the motors, loads params, starts the thread. Motor stays off |
| `setTargetVelocity(ticksPerSec)` | ≤ 0 means coast |
| `setTargetRpm(rpm)` | Flywheel RPM, using `ticksPerRev` |
| `off()` | Coast (power 0, `FLOAT`) |
| `close()` | Stop the thread and the motors. Safe to call twice |
| `isReady()` | Within `max(readyTolFrac · target, readyTolMin)` of target for `readySettleMs` |
| `getVelocity()`, `getTarget()`, `getMode()` | Latest values |
| `notifyShot()` | Optional: a ball was just pushed toward the wheel. See below |
| `getShotCount()` | Balls detected from speed dips |
| `getShotLeadSec()`, `getShotDropFrac()`, `getShotLearnCount()` | What `notifyShot()` has learned |
| `getState()` | Full snapshot: target, velocity, estimate, load, power, battery, loop/solve time, ready, shot flags |
| `getError()` | The exception that killed the control thread, or `null` |
| `getParams()` / `setParams(p)` / `saveParams(comment)` | Read, change and persist tuning at runtime |
| `startLogging(n)`, `getLog()`, `getLogSince(t)`, `dumpLog(prefix)` | Per-loop ring buffer and CSV export |
| `setOpenLoopPower(p)` | Raw power, no control (SysId uses it) |
| `virtualShot(seconds)` | Test tool: cut the motor without telling the controller (Auto-Tune uses it) |

## Shooting: recovery and `notifyShot()`

### Recovery is automatic

You don't program recovery. When a ball goes through, the launcher detects the drop, plans more power immediately, and doesn't over-correct afterwards. Your code only decides **when to feed the next ball: when `isReady()` is true.** Holding a feed button gated on `isReady()` gives rapid fire at exactly the rate the launcher can recover.

**Shoot N balls** (autonomous or a "shoot 3" button), feeding each only after the last one was seen:

```java
int startShots = launcher.getShotCount();
int fired = 0;
ElapsedTime timeout = new ElapsedTime();

while (opModeIsActive() && fired < 3 && timeout.seconds() < 4) {
    if (launcher.isReady() && launcher.getShotCount() == startShots + fired) {
        feeder.push();
        launcher.notifyShot();
        fired++;
    }
    // ...rest of your loop
}
```

The check `getShotCount() == startShots + fired` stops two balls going into one dip. Keep the timeout for jams: if a ball never reaches the wheel, the count stops going up.

### `notifyShot()`: start the recovery as the ball hits

Without it, the controller only reacts once the encoder shows the drop, 20–30 ms into it. Your code knows earlier, because it pushes the feeder. Call `launcher.notifyShot()` **at the moment you command the feeder**. You don't need to know how long the ball takes to reach the wheel:

1. **The first 2 notified balls only teach it.** For each, it measures the time from `notifyShot()` to the dip starting (the "lead") and how much speed the ball took.
2. **From the 3rd ball on**, it starts the full recovery push the moment the ball is expected to hit, instead of 20–30 ms later.
3. It keeps learning (a moving average), so slow changes are followed.
4. The learned values go into the params file the next time anything saves it (Y in the tuner, or Auto-Tune), so it doesn't relearn every match. They're also shown in the Tuner as `notifyShot learned: …`.

If a notified ball never arrives within 1 s (a jam or a missed feed), it's forgotten and nothing is learned from it.

**What it does and doesn't do** (simulation, 150 ms feeder delay, balls taking 12% of speed):

| | Without | With `notifyShot()` |
|---|---|---|
| Dip depth | ~155 ticks/s | ~115 ticks/s (25% shallower) |
| Recovery time | ~180 ms | ~170 ms |
| Over-speed before / after the ball | none | none |
| Learned lead | — | within 3 ms of the true value |

It can't remove the dip. A ball takes ~10% of the speed in ~30 ms, and the motor's spare power can only add back about 1% in that time. The only way to "prevent" the dip would be spinning faster before the ball arrives, and that sends that ball long. An earlier version did exactly that; [launcher-internals.md](launcher-internals.md#shot-feed-forward-notifyshot) explains why it was removed.

`shotFeedforward` (default 1) scales how much of the expected dip it plans for. Set it to 0 to switch the feature off without removing the calls.

## Parameter reference

All are in [`LauncherParams.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/LauncherParams.java) and saved to the `.properties` file. Out-of-range values are clamped.

### Model (set by SysId; don't hand-tune)

| Parameter | Default | Meaning |
|---|---|---|
| `tau` | 0.60 s | Time constant |
| `kV` | 2400 | Ticks/s per unit power at `nominalVoltage` |
| `kS` | 80 | Friction offset, ticks/s |
| `delaySec` | 0.010 | Dead time between command and encoder |
| `nominalVoltage` | 12.0 | Voltage the model is normalised to |

### Controller weights (Auto-Tune or by hand)

| Parameter | Default | Raise it when… | Lower it when… |
|---|---|---|---|
| `rDu` | 0.05 | The motor buzzes or power is jumpy | Steps or recovery are sluggish |
| `loadNoise` | 4 | Shot recovery is slow, or the speed sits off target after a change | The speed wanders while holding |
| `measNoise` | 30 | Power is noisy while holding speed; false shot detections | Reactions feel late |
| `velNoise` | 6 | (rarely) the estimate lags real speed changes | (rarely) the estimate is jumpy |
| `qTerminal` | 5.0 | Overshoot at the end of steps | (rarely matters) |
| `horizonSec` | 0.60 (≈ tau) | (rarely) slow approach to target | (rarely) |
| `horizonSteps` | 15 | Finer plan, more CPU | Solve time too long on the hub |
| `qVel` | 1.0 | Leave at 1; the others are relative to it | |

### Shots

| Parameter | Default | Meaning |
|---|---|---|
| `shotDetectSigma` | 3 | Shot detector threshold. Lower = catches smaller balls; higher = fewer false alarms; 0 = off |
| `shotFeedforward` | 1.0 | `notifyShot()` push at contact: 1 = full, 0 = off |
| `shotLeadSec` | 0 (learned) | Time from `notifyShot()` to the ball hitting. Learned; 0 = not yet |
| `shotDropFrac` | 0 (learned) | Speed lost per ball, fraction. Learned; 0 = not yet |

### Limits, loop and "ready"

| Parameter | Default | Meaning |
|---|---|---|
| `uMin` / `uMax` | 0 / 1 | Power limits. A small negative `uMin` (e.g. −0.2) lets it brake when slowing down |
| `maxDuPerSec` | 0 (off) | Extra slew limit on power, per second |
| `periodMs` | 10 | Control period (100 Hz) |
| `maxIters` | 40 | Solver iterations per loop. Lower if the hub's solve time is too long |
| `readyTolFrac` / `readyTolMin` | 0.02 / 20 | Ready band: `max(2% of target, 20 ticks/s)` |
| `readySettleMs` | 80 | Time in the band before `isReady()` is true |
| `ticksPerRev` | 28 | For `setTargetRpm` and RPM display only |

Defaults come from simulator sweeps. On the real launcher, SysId and Auto-Tune have the final say.

## Tools

### Desktop simulator

[`tools/launcher/sim.sh`](../tools/launcher/sim.sh) compiles the real controller classes (plain Java with no FTC imports) and runs them against a simulated flywheel. The simulation has encoder noise, dead time, loop-timing jitter, battery sag, model error, and 30 ms ball contacts.

```bash
./tools/launcher/sim.sh run                                    # spin-up, step down, step up, 3 shots
./tools/launcher/sim.sh run launcher.properties --csv run.csv  # your robot's params, write a CSV
./tools/launcher/sim.sh run --shots 6 --notify 0.15            # notifyShot() with a 150 ms feeder delay
./tools/launcher/sim.sh sweep launcher.properties              # rank weight combinations, robust to ±10% model error
./tools/launcher/sim.sh sysid --mismatch 0.2                   # check SysId recovers a known launcher
./tools/launcher/sim.sh autotune launcher.properties           # run the Auto-Tune search on a simulated launcher
./tools/launcher/sim.sh autotune --bad --noise 30              # start from bad values and watch it recover
```

| Option | Meaning |
|---|---|
| `--mismatch X` | Real launcher off from the model by ±X (e.g. 0.15 = 15%) |
| `--shots N`, `--shotDrop X` | Number of balls; speed each takes (default 0.12) |
| `--notify L` | Call `notifyShot()` L seconds before each ball hits (±15 ms), timing unknown to the controller |
| `--target T`, `--noise S`, `--seed N` | High target, encoder noise std-dev, random seed |
| `--bad`, `--trials N` | For `autotune`: start from deliberately bad weights; search budget |

Use `adb pull /sdcard/FIRST/launcher/launcher.properties` to get the robot's params. The sim prints the same metrics and score as the tuner (same code), so desk and robot numbers compare directly.

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
| Motor buzzes while holding speed | `rDu` or `measNoise` too low |
| Slow recovery after shots | Target too close to top speed (most common). Then raise `loadNoise` or lower `rDu`. Add `notifyShot()` |
| Next ball goes long after a shot | Shots not being detected (watch `shots detected`). Lower `shotDetectSigma` |
| `shots detected` climbs with no balls | Raise `shotDetectSigma` or `measNoise` |
| `notifyShot learned` stays at 0 | Balls aren't being detected within 1 s of the call, or the call is made after the ball has already hit |
| Auto-Tune says "already the best" every time | Normal if SysId was run and the defaults suit your launcher. Confirm with real balls in the Tuner |
| Tuner shows `CONTROL THREAD DIED` | See the message. Usually a hardware name missing from the config |
| `Direct bulk read: NO (fallback)` in SysId | The motor's hub couldn't be found. Speed falls back to `getVelocity()`, which may be stale if the main thread uses `MANUAL` caching |
| Solve time over ~2 ms | Lower `horizonSteps` (e.g. 10) or `maxIters` (e.g. 25) |

## Changes from last year's launcher

| DECODE (2025-26) | Now |
|---|---|
| Brute-force search over 9 power sequences (±0.05 steps, 2-step horizon) | Real MPC: 15-step horizon, constrained QP |
| Ran inside the OpMode loop, so its rate depended on everything else | Own thread at a fixed 100 Hz with its own bulk read |
| `kI` integral added after the MPC, with a 2% gate | Kalman load estimate. No integral to tune or wind up |
| No battery compensation | Battery voltage is part of the model |
| Model fitted by hand from telemetry (`kS` came out negative) | `Launcher SysId` fits it automatically, unbiased by encoder noise |
| Shots: integral reset, slow recovery | Shot detection, no over-speed after recovery, shot counting, optional `notifyShot()` |
| Tune by editing `TeleConstants` and rebuilding | Auto-Tune, or tune live from the gamepad; saved on the Control Hub |

## Not yet verified on the robot

Everything here is checked in the desktop simulator and compiles against SDK 12. None of it has run on hardware yet:

- **Solve time on the Control Hub.** It's shown in the tuner as `solve`. If it's over ~2 ms, lower `horizonSteps` or `maxIters`.
- **Self-stop when the OpMode ends.** The thread checks for this, but it hasn't been confirmed on a hub. Until it has, call `launcher.close()` at the end.
- **Launcher Auto-Tune.** On the first real run, check in the Tuner that virtual shots and real balls produce similar `DIP` depths, then confirm the saved values with real balls.
- **`notifyShot()`.** Check `notifyShot learned` shows a sensible lead (roughly the feeder's travel time) after a couple of balls.
