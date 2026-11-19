# Launcher MPC

Flywheel velocity control with model-predictive control (MPC), running on its own thread at 100 Hz.

```java
LauncherMPC launcher = new LauncherMPC(this);   // init: thread starts, motor stays off
launcher.setTargetVelocity(1800);               // ticks/s; no update() call needed
if (launcher.isReady()) feed();                 // at speed and settled
launcher.off();                                 // coast
```

The thread stops and zeroes the motors on its own within ~50 ms of the OpMode ending. You can also call `launcher.close()`.

## What changed from last year's `LauncherMPC`

| DECODE (2025-26) | Now |
|---|---|
| Brute-force search over 9 power sequences (±0.05 steps, 2-step horizon) | Real MPC: 15-step horizon over ~tau, box-constrained QP solved with warm-started FISTA (~40 µs) |
| Ran inside the OpMode loop, so its rate depended on everything else | Own thread at a fixed 100 Hz with its own bulk read |
| `kI` integral bolted on after the MPC, with a 2% gate | Kalman filter estimates speed and load, so tracking has no steady-state offset and there is no integral to wind up |
| No battery compensation | Model uses the battery voltage, so the first shot and the 30th match the same |
| Model fit by hand from telemetry (`kS` came out negative) | `Launcher SysId` fits tau, kV, kS and dead time automatically, using an output-error fit that encoder noise doesn't bias |
| Shots: integral reset, slow recovery | Shot detector: a ball's speed drop isn't mistaken for load, so there's no over-speed after recovery. Shots are also counted |
| Tuning = edit `TeleConstants`, rebuild | Tune live from the gamepad; it saves to a file on the Control Hub with no rebuild |

## Files

| File | What |
|---|---|
| `LauncherMPC.java` | Hardware, thread, public API. **Edit `defaultConfig()` for motor names and directions.** |
| `MpcCore.java` | Observer, delay compensation and QP solver. Pure Java, shared with the simulator |
| `LauncherParams.java` | Every tunable value, plus save/load of `/sdcard/FIRST/launcher/launcher.properties` |
| `SysIdFit.java` | Model fitting |
| `StepAnalyzer.java`, `TuneScore.java` | Response metrics and the single tuning score (robot and sim use the same code) |
| `opmodes/tuning/LauncherSysId.java` | Step 1: measure the launcher |
| `opmodes/tuning/LauncherMPCTuner.java` | Step 2: tune live |
| `tools/launcher/sim.sh` | Desktop simulator, weight sweep and SysId check |
| `tools/launcher/plot.html` | Open in a browser and drop in a CSV log to plot it |

## Tuning recipe (new launcher, about 15 minutes)

0. **Config.** In `LauncherMPC.defaultConfig()`, set the motor names and directions so that positive power spins the wheel the shooting way. Use `.encoder(i)` to say which motor has the encoder cable.
1. **Launcher SysId** (tuning group). Remove balls, start with a battery at 12.5 V or more, press start. It runs about 30 s up to full power, then shows `tau`, `kV`, `kS`, delay and the fit quality.
   - You want r² > 0.99. The steady-state check should agree with the fitted `kV` within 10%.
   - The **top speed** it prints is the physical limit at 12 V. Keep match targets at or below 85% of it. Without that headroom, nothing can recover from a shot fast.
   - Press **A** to save. This also sets `horizonSec ≈ tau` and `measNoise` from the measured noise.
2. **Launcher MPC Tuner**. Set `targetHigh` and `targetLow` to your real shooting speeds.
   - Press **X** for the auto test (off, HIGH, LOW, HIGH). **Feed 3 balls during the second HIGH.** It prints `AUTO SCORE` (lower is better). Change one parameter, run X again, keep whatever lowers the score.
   - Adjust in this order:
     1. **`rDu`**: response speed against smoothness. Lower it until spin-up stops getting faster or `chatter` passes ~0.008/loop (the motor starts to buzz).
     2. **`loadNoise`**: shot recovery. Raise it if `DIP … recover` is slow. Lower it if the speed wanders while holding.
     3. **`measNoise`**: raise it if power is noisy while holding speed.
     4. **`shotDetectSigma`**: watch `shots detected` while feeding balls. Lower it if shots are missed. Raise it if it counts shots while nothing is fed.
     5. `horizonSec` and `qTerminal` rarely matter for a flywheel.
   - Press **Y** to save. Every OpMode that creates `LauncherMPC` loads the file at init.
3. If something looks odd, press **back** to write a CSV (`/sdcard/FIRST/launcher/tune-*.csv`), then plot it:

   ```bash
   adb pull /sdcard/FIRST/launcher/ launcher-logs
   ```

   Open `tools/launcher/plot.html` and drop the file in.

Re-run SysId whenever the mechanism changes (wheel, gearing, compression, belt).

## Desktop simulator

It runs the exact controller code against a simulated flywheel with noise, delay, battery sag, model error and 30 ms ball contacts.

```bash
./tools/launcher/sim.sh run                                   # default params, standard scenario
./tools/launcher/sim.sh run launcher.properties --mismatch 0.15 --csv run.csv
./tools/launcher/sim.sh sweep launcher.properties             # best rDu / noise settings, robust to ±10% model error
./tools/launcher/sim.sh sysid --mismatch 0.2                  # check SysId recovers a known plant
```

Pull the robot's file with `adb pull /sdcard/FIRST/launcher/launcher.properties`. Results with the default model (tau 0.6 s, 15 ticks/s encoder noise):

- Spin-up 0 to 1624 ticks/s: rise 0.6 s (full power the whole way, so physics-limited), overshoot 1%.
- A ball that takes 12% of the speed: back within 2% in ~0.17 s, over-speed afterwards under 1.1%.
- Model off by 15–30%: still no steady-state error, just slower.
- Solve time: ~40 µs per loop on a laptop. Expect roughly 5–10× that on the Control Hub (not measured on a hub yet). The tuner shows the real `solve` time.

## Threading notes

- Velocity comes from `LynxModule.getBulkData()` on the launcher's own hub. That's a fresh bulk read that also refreshes the hub's cache, so main-thread code using MANUAL bulk caching (like `MecanumDrive`) keeps working and just sees slightly newer data.
- Each loop costs one bulk read, plus one `setPower` per motor (skipped when the change is under 0.2%), plus a battery read at 10 Hz.
- Every getter returns a snapshot (`getState()`) published by the thread. Every setter only writes a volatile. Nothing blocks the OpMode loop.
