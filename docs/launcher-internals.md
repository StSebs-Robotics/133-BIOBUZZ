# Launcher MPC: How It Works

The design, the maths, and the reasons behind each decision, including what the simulator showed and the approaches that didn't work. For setup, tuning and usage see **[launcher.md](launcher.md)**.

**Contents:** [Code map](#code-map) · [The control loop](#the-control-loop) · [Model and battery compensation](#model-and-battery-compensation) · [Kalman filter](#kalman-filter) · [Delay compensation](#delay-compensation) · [MPC](#mpc) · [Shot detection](#shot-detection) · [Shot feed-forward (`notifyShot`)](#shot-feed-forward-notifyshot) · [Threading and hub access](#threading-and-hub-access) · [System identification](#system-identification) · [Scoring](#scoring) · [Auto-Tune](#auto-tune) · [How it was verified](#how-it-was-verified) · [Ideas not done](#ideas-not-done)

## Code map

All in [`subsystems/launcher/`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/) unless noted. The **pure** classes have no FTC imports, so the desktop simulator compiles and runs exactly the code the robot runs.

| Class | Pure | Role |
|---|---|---|
| `LauncherMPC` | | Hardware, the control thread, public API, logging, safety |
| `MpcCore` | ✓ | Kalman filter, shot detection and learning, delay compensation, QP build and solve |
| `LauncherParams` | ✓ | Every tunable number; load/save `.properties`; clamping |
| `SysIdFit` | ✓ | Model fitting from open-loop data |
| `StepAnalyzer` | ✓ | Turns a speed trace into step and shot metrics |
| `TuneScore` | ✓ | One score from those metrics (the Tuner's `AUTO SCORE`, the sim's `score`) |
| `AutoTuneTrial` | ✓ | One Auto-Tune test sequence and its score |
| `AutoTuner` | ✓ | The Auto-Tune search |
| `opmodes/tuning/LauncherSysId`, `LauncherAutoTune`, `LauncherMPCTuner` | | The three tuning OpModes |
| `tools/launcher/LauncherSim.java` | | Simulated flywheel and the `run` / `sweep` / `sysid` / `autotune` modes |

## The control loop

`LauncherMPC.run()` runs on a dedicated thread (`MAX_PRIORITY − 1`) every `periodMs` (10 ms):

```
 ┌──────────────┐  ┌──────────────────┐  ┌────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────┐
 │ bulk read    │─▶│ Kalman filter    │─▶│ delay      │─▶│ MPC: plan    │─▶│ setPower     │─▶│ publish  │
 │ speed        │  │ speed + load,    │  │ compensa-  │  │ 15 steps,    │  │ (skipped if  │  │ State,   │
 │ battery@10Hz │  │ shot detect/learn│  │ tion       │  │ send step 1  │  │ <0.2% change)│  │ log row  │
 └──────────────┘  └──────────────────┘  └────────────┘  └──────────────┘  └──────────────┘  └──────────┘
```

- **Fixed rate:** the next wake-up is `next += period`, slept with `LockSupport.parkNanos`. If a loop overruns, the schedule resets instead of bursting to catch up.
- **Real `dt`:** the filter uses the measured time since the last loop (it jitters by a few ms); the MPC plans on the nominal grid.
- **Modes:** `OFF` (power 0, plan reset), `VELOCITY` (the MPC), `OPEN_LOOP` (raw power for SysId). The filter runs in every mode, so the estimate is valid the moment you switch to `VELOCITY`.
- **Parameter changes** (`setParams`) are handed over through a volatile field and picked up at the start of the next loop, so a change never lands half-way through a solve.

## Model and battery compensation

```
tau · dv/dt = −v + kV · w − kS − load          w = power · Vbattery / Vnominal
```

- `v`: wheel speed (ticks/s); `power`: −1..1; `w`: the effective voltage fraction actually reaching the motor.
- **First-order** because a flywheel driven by a DC motor is, to a good approximation, one inertia with viscous-like back-EMF damping. The motor's electrical time constant (~1 ms) is far faster than the mechanical one (~0.5 s) and is absorbed into the dead time.
- **Battery:** motor drive is PWM, so the average motor voltage is `power × battery`. Normalising by the battery makes `kV` mean "at 12 V" whatever the battery was during SysId, and makes the MPC ask for more power on a tired battery automatically. The battery is read from the hub's input voltage 10 times a second and smoothed (`b += 0.3·(new − b)`), because it's a separate hub transaction. A sag faster than that is seen up to ~0.1–0.3 s late; the load estimate absorbs the difference meanwhile.
- **Friction floor:** in the predictions, friction can't spin the wheel backwards. A coasting prediction is floored at 0.
- **Discretised** exactly for a step of length `h`: `v⁺ = a·v + (1 − a)(kV·w − kS − load)`, with `a = e^(−h/tau)`.

## Kalman filter

**State** `x = [v, load]`, with load modelled as a random walk:

```
v⁺    = a·v − (1−a)·load + (1−a)(kV·w − kS)
load⁺ = load
y     = v + noise
```

**Gains:** steady-state Kalman gains, computed once per parameter change by iterating the 2×2 Riccati equation to convergence (`computeKalmanGains`), with:

- `Q = diag(velNoise², loadNoise²)`
- `R = measNoise²`

Each loop: predict with the real `dt` and the input that actually acted (from the delay history), then correct both states by `K · innovation`.

**What the three noises mean:** only their ratios matter. Scaling all three by the same factor gives the same gains.

| Ratio | Effect |
|---|---|
| `loadNoise / measNoise` | How fast load is learned: offset removal and adapting to friction changes |
| `velNoise / measNoise` | How much the speed estimate follows the encoder versus the model |

**Why it removes steady-state error:** any persistent difference between model and reality shows up as a persistent innovation. That drives the load estimate until the model's prediction matches, and the MPC then plans with the corrected load. This is the "disturbance observer" form of offset-free MPC, and replaces last year's integral term (which needed a 2% gate and anti-windup). In the simulator the speed settles exactly on target with the model 30% wrong.

The load estimate is clamped to ±2·`kV`, so a fault can't wind it up without limit.

## Delay compensation

A command sent now acts after `delaySec` (SysId measures it: hub latency plus the hub's velocity filtering). With `nd = round(delaySec / period)`, the core keeps the last `nd + 1` effective inputs:

- `hist[0]` is the input that acted over the interval that just ended; the filter uses it.
- `hist[1..nd]` are sent but not yet visible. Before solving, the estimate is rolled forward through them (a Smith predictor), so the MPC plans from the speed the wheel will have when this command lands.

In the simulator, a real delay of 4× the modelled one still worked (offset-free, slightly slower settling).

## MPC

**Prediction grid:** `N = horizonSteps` steps of `tp = horizonSec / N` (default 15 × 40 ms = 0.6 s), separate from the 10 ms control period. A horizon of about one `tau` is needed to see where a power change leads; 60 steps of 10 ms would be a 60-variable QP for no gain. Only the first planned power is sent, and the plan is recomputed every 10 ms.

**Prediction is linear in the power plan `u`:** `v = s + G·u`, where:

| Term | Meaning |
|---|---|
| `s_j` | Free response: speed with zero power, from the delay-compensated start, including friction and the current load estimate. Also where a known coming shot is added (see feed-forward) |
| `G[j][i] = a^(j−i) · b` | For `i ≤ j`, else 0. `b = (1−a)·kV·Vbattery/Vnom` is the speed gained per unit power per step |

**Cost:**

```
J(u) = Σ_j q_j · ((target − v_j) / kV)²  +  rDu · Σ_j (u_j − u_{j−1})²
q_j = qVel, plus qTerminal on the last step;  u_{−1} = the power sent last loop
subject to uMin ≤ u_j ≤ uMax
```

Dividing the speed error by `kV` expresses it in "power units", so the weights are dimensionless and carry over between launchers with different gearing or encoders.

**As a QP:** `J = ½uᵀHu + fᵀu + const`, with:

- `H = (2/kV²)·GᵀQG + 2·rDu·DᵀD`, where `D` is the first-difference matrix (`DᵀD` is tridiagonal);
- `f = (2/kV²)·GᵀQ(s − target) − 2·rDu·u_{−1}·e₀`.

`H` is rebuilt every loop (it depends on the battery through `b`). That's ~N³/6 ≈ 560 multiply-adds, which is trivial.

**Solver:** FISTA (accelerated projected gradient):
- step `1/L`, where `L` is a Gershgorin bound on `H`'s largest eigenvalue (the maximum absolute row sum: safe and cheap);
- projection onto the box `[uMin, uMax]` is a clamp;
- gradient-based adaptive restart (O'Donoghue & Candès) prevents the oscillation accelerated methods can have;
- warm start from the previous plan; stop when no element changes by more than 1e-7, or after `maxIters` (40).

A box-constrained QP is exactly what this solver handles well, with no matrix factorisation and fixed memory. Checked against 20 000-iteration solutions, 40 warm-started iterations match the first planned power to within 0.003 (0.3% power) in the worst case tested.

**Speed:** ~30–80 µs per loop on a laptop. The Control Hub hasn't been measured yet; the tuner shows `solve` live.

**Notes:**
- `qTerminal` and `horizonSec` barely matter for a first-order flywheel: the weight sweeps found the score almost flat in both.
- `maxDuPerSec` (off by default) is applied after the QP as a plain slew limit, not inside it. Rate constraints inside the QP would make the feasible set non-box and the solver much more complex.

## Shot detection

A ball takes ~10% of the speed in ~30 ms. To a plain Kalman filter that looks like a sudden load increase. It then raises the load estimate, the MPC keeps adding power after the speed has recovered, and the wheel over-speeds for about a second. In the simulator that over-speed was 2–4%, enough to send the next ball long.

**Detector:** a one-sided CUSUM on how much slower than predicted the wheel is:

```
z = −innovation / measNoise          (positive when slower than the model says)
g = max(0, g + z − 1)                (drift of 1σ: noise alone keeps g near 0)
shot detected when g > shotDetectSigma
```

**When a shot is detected:**
- The speed estimate follows the encoder directly. It's a speed jump, so the model's prediction is irrelevant.
- The load estimate is put back to its value from just before the drop began (recorded when `g` first left 0), undoing what the first samples of the dip did to it.
- The shot ends when the speed stops falling (2 samples with `z < 1`), or after 0.2 s at most. The cap means a real, lasting load change can never be locked out of the filter.

**Armed only near the target** (within 3× the ready band). During spin-up with a wrong model, the wheel is also consistently "slower than predicted"; an unarmed version of the detector froze the load estimate there and stopped the speed reaching its target (simulator, 15% model error). Balls are only fired while holding speed anyway.

**Result (simulator):** post-shot over-speed went from 1.9–3.6% to 0.6–1.1%, with spin-up and steps unchanged.

## Shot feed-forward (`notifyShot`)

**Learning.** `notifyShot()` timestamps the feed command. When the next shot is detected within 1 s, two things are measured:
- **lead**: the time from the notify to the moment the dip started;
- **drop**: the speed lost, from the speed at the dip start to the minimum of a 3-sample moving average during the shot (raw minimums would be biased by noise).

Both are averaged: a plain mean over the first 4 shots, then a moving average with weight 0.3. After `LEARN_SHOTS = 2` it's active. Learned values are copied into `getParams()` so any save persists them, and they're re-loaded from the file at init.

**Use.** While a notified ball is pending, the MPC's free response gets a predicted dip of `shotFeedforward · drop · target` at the expected contact time, decaying like any speed error. The QP then plans the power to cover it. Once the real shot is detected, the prediction is dropped and the measurement takes over.

**Why the dip is only planned for once the command lands at or after contact.** The first version put the predicted dip into the plan as soon as the notify arrived. The simulator showed:
- dips halved and recovery 3× faster, which looked great;
- but the wheel was **3–5% over speed when each ball hit**, so every ball after the learning shots would leave fast.

The MPC had, correctly, found that the cheapest way to reduce squared error after the dip is to speed up before it. Weighting the pre-contact steps 20× to forbid that made the QP too stiff for 40 iterations (power alternating 0.7 / 1.0) and still gave 3% over speed.

The physics explains why: a ball removes ~10% of the speed in ~30 ms, while the motor's spare power can add back only ~1% in that time. **No controller can prevent the dip without pre-spinning.** So the feed-forward is limited to what's physically useful: start the full recovery push the moment the ball hits, instead of 20–30 ms later when the detector sees it.

**Result:** dips 25% shallower (~155 → ~115 ticks/s), no over-speed at all, and the lead learned to within 3 ms at 80 ms, 150 ms and 300 ms feeder delays.

## Threading and hub access

- **Velocity read:** `LynxModule.getBulkData()` on the encoder motor's own hub, a fresh bulk read every loop. The hub is found through `LynxController.getModule()`, which is protected and reached by reflection, because the Control Hub and an Expansion Hub share one USB serial number and can't be told apart by serial. If that fails, it falls back to `motor.getVelocity()` (and SysId shows "fallback").
- **Bulk cache:** `getBulkData()` also refreshes the hub's cache. Main-thread code in `MANUAL` mode (like `MecanumDrive`) just gets slightly newer data.
- **Velocity sign:** raw bulk data is in the motor's native direction. The sign is computed the way the SDK does: the configured direction, flipped if the motor type's orientation is CCW.
- **Writes:** through `CachedMotor` with a 0.2% threshold (each `setPower` is a blocking hub round trip of ~1–3 ms). No voltage compensation there: the model does it.
- **Snapshots:** each loop publishes an immutable `State` through a volatile field, so readers never see a half-updated set of values. Commands are single volatile writes.
- **Safety:** every 50 ms the thread checks that `OpModeManagerImpl.getActiveOpMode()` is still its owner OpMode. It only trusts that check after it has seen the owner active once, so an SDK difference can never stop the launcher at init. On owner change, `close()`, interrupt or any exception, it sets power 0 and exits. `getError()` keeps the exception.
- **Log:** a ring buffer of float rows (`t, target, velocity, estimate, load, power, battery, loopMs, solveMs`). `getLogSince(t)` returns only new rows, so the tuners can score every 10 ms sample from a slower OpMode loop.

## System identification

`SysIdFit` uses an **output-error** fit:
- for a candidate `tau` and dead time, simulate the model over the whole log;
- the simulated response is linear in `kV` and `kS`, so solve those by least squares;
- golden-section search on `log(tau)`, for each dead time from 0 to 100 ms; keep the best.

Samples below 8% of the maximum speed are excluded (friction there isn't linear).

**Why not the obvious one-step regression** (`v[k+1] = a·v[k] + β·w + γ`)? Encoder noise sits on the regressor `v[k]`, which biases `a` low (errors-in-variables). On the simulated launcher it gave `tau` 0.51 instead of 0.60, `kS` 27 instead of 80, and no delay. The output-error fit gave `tau` 0.592, `kV` 2402 (true 2400), `kS` 81.3 (true 80). It stayed within ~1.5% with a 20% different launcher and with double the noise.

A separate straight-line fit of the settled `(w, v)` points gives an independent `kV`/`kS`. If the two disagree by more than 10%, something nonlinear is going on (a slipping belt, a sagging battery).

## Scoring

**`StepAnalyzer`** watches `(t, target, speed)`:
- **Steps:** a target change starts a 2 s window. It reports rise (10→90%), overshoot, settle time, RMS error over the last quarter, and IAE normalised by the step size.
- **Dips:** while holding speed, a drop below `max(2·band, 3% of target)` starts a dip. It reports the depth, the recovery time back into the band, and the over-speed in the 0.6 s after recovering.
- **Smoothing:** speed is filtered at 20 ms first, so metrics describe the wheel and not the encoder noise. This adds ~20 ms to the reported rise time.

**`TuneScore`** (Tuner, sim):

```
Σ steps (IAE + 0.02·overshoot% + 0.01·ssRms + [1 if ssRms > band])
+ Σ dips (recovery s + 0.05·over-speed%)
+ 20·chatter
```

Coast-down steps to 0 aren't scored.

**`AutoTuneTrial`** uses a variant without the step overshoot term, and with a smooth off-target penalty `max(0, ssRms/band − 1)`.

Lessons from measuring the scores themselves:
- **On game-sized steps the "overshoot" is about the size of the encoder noise.** It made trial scores noisy, and IAE already includes real overshoot.
- **Hard +1 penalties flip on and off with noise.** The original "+1 if not settled" (decided by single noisy samples) made identical settings score ±40% apart. After the fixes, trial-to-trial spread is ~2–4%.

## Auto-Tune

**Virtual shots.** Without shots in the test, the search learned to be calm (`measNoise` up to 300). That's best for steps and buzz but ruins recovery: two of four simulated cases got worse on the real-shot scenario. Real balls can't be fed automatically, so the motor is cut for 0.1·`tau` while `MpcCore` is told the power went through. That's an unexplained speed loss, the same as a ball.

**Search.** Coordinate descent in log space over `rDu`, `loadNoise` and `measNoise` (the three that matter most and interact least), with ×2 steps:
- ×√2 steps changed the score less than the noise, so a finer pass made no progress;
- passes repeat until one changes nothing (max 3).

**Handling noise and drift** (each found in the simulator):

| Problem | Fix |
|---|---|
| The first baseline ran before the filter had settled after spin-up, and was unrepresentative (4.7 against ~7 for the same settings later) | One unscored warm-up trial |
| Comparing against an old score lets drift decide | Re-measure the best before each parameter; compare against the mean of its last 3 scores |
| Single trials can mislead | 3% margin; close calls get a second trial and are judged on the average |
| A lucky trial could still win | Final check: winner and start alternately, twice each; save only if the winner's mean is ≥3% better |

**Results (simulator):**

| Case | Outcome |
|---|---|
| Sensible start (defaults), two seeds | Correctly no change |
| Very noisy encoder, bad start | Fixed; real-shot score 3.35 → 2.56 |
| Model 15% off | Lowered `rDu`; real-shot score within noise (1.73 → 1.77) |
| Bad start with 15% model error | No change: one 13 s trial couldn't tell what was wrong |

It never saved a change that made things clearly worse after the double check was added.

## How it was verified

| What | How | Result |
|---|---|---|
| Compiles against FTC SDK 12 | `./gradlew :TeamCode:compileDebugJavaWithJavac` | Pass |
| QP solver accuracy | 40 warm-started iterations vs 20 000 | First power within 0.003 |
| Tracking with model error | Sim, ±15% and 30% model error | No steady-state error |
| Delay robustness | Sim, real delay 4× the model | Works, slightly slower settling |
| SysId accuracy | Sim, known launchers, 1× and 2× noise | `tau`/`kV`/`kS` within ~1.5% |
| Shot detection | Sim, 5–25% balls over 30 ms | Post-shot over-speed ≤ ~1% (from 2–4%) |
| `notifyShot` learning | Sim, 80/150/300 ms feeder delay, ±15 ms jitter | Lead within 3 ms; dips 25% shallower; no over-speed |
| Auto-Tune | Sim, 6 scenarios | See above |
| CSV plotter | Loaded a sim log in a browser | Plots correctly |
| **On the robot** | — | **Not yet:** see [launcher.md](launcher.md#not-yet-verified-on-the-robot) |

The simulator's flywheel isn't the real one. It's a first-order wheel with integer-quantised noisy speed, latency, loop jitter, battery sag (12.6 V − 1.2 V × power), static friction, and ball contacts that remove a fraction of speed over 30 ms. Real launchers add things it doesn't model: compression variation between balls, belt stretch, and motor heating.

## Ideas not done

- **Re-fit the model online** (slowly updating `tau`/`kV` during matches). The load estimate already absorbs most drift; a full re-fit adds risk mid-match.
- **Dual-motor balancing**, if the two motors of a flywheel ever fight. Today both get the same power.
- **Per-distance targets** (a speed table from Limelight distance). That belongs in the shooting code, not the controller; `setTargetVelocity` can be called every loop.
- **Panels graphs.** Panels is installed with Pedro now; the Tuner could stream target/velocity/power live instead of needing a CSV.
