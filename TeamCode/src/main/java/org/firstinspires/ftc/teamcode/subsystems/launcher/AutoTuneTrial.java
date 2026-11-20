package org.firstinspires.ftc.teamcode.subsystems.launcher;

import java.util.List;

/**
 * One auto-tune trial: a fixed test sequence and its score. Pure Java (shared by the Launcher
 * Auto-Tune OpMode and the simulator).
 *
 * <pre>
 *   [settle at LOW, not scored] → HIGH → LOW → HIGH → LOW      (cycles × 2 holds)
 *   in every hold, after the step has been scored: one VIRTUAL SHOT
 * </pre>
 *
 * Virtual shot: the motor is cut for {@link #shotSec} while the controller is told its power went
 * through as normal ({@link LauncherMPC#virtualShot}). To the controller that is an unexplained
 * speed loss, exactly like a ball, so shot recovery is tested without anyone feeding balls.
 * Without it the tuner learns to be calm and slow, which ruins real shot recovery.
 *
 * Use a step size like the real game (LOW ≈ 85% of HIGH): big steps run at full power the whole
 * way and say little about the tuning. Starts and ends at LOW, so trials run back to back.
 */
public final class AutoTuneTrial {

    /** Steps are scored over 2 s (StepAnalyzer window); the shot comes after that. */
    private static final double SHOT_AFTER_SEC = 2.1;
    /** Time after the shot for recovery + over-speed check before the next step. */
    private static final double AFTER_SHOT_SEC = 0.9;

    public final double low, high, settleSec, holdSec, shotSec;
    public final int cycles;
    private final TuneScore score;
    private final double tolFrac, tolMin;

    /**
     * @param shotSec virtual shot length (motor cut), s. See {@link #defaultShotSec}. 0 = no shots.
     */
    public AutoTuneTrial(double low, double high, double settleSec, int cycles, double shotSec,
                         double tolFrac, double tolMin) {
        this.low = low;
        this.high = high;
        this.settleSec = settleSec;
        this.cycles = cycles;
        this.shotSec = Math.max(shotSec, 0);
        this.holdSec = SHOT_AFTER_SEC + this.shotSec + AFTER_SHOT_SEC;
        this.score = new TuneScore(tolFrac, tolMin);
        this.tolFrac = tolFrac;
        this.tolMin = tolMin;
    }

    /** Virtual shot length that loses about 10% of speed for this model: 0.1·tau, 20–150 ms. */
    public static double defaultShotSec(LauncherParams p) {
        return Math.max(0.02, Math.min(0.15, 0.1 * p.tau));
    }

    public double durationSec() {
        return settleSec + 2 * cycles * holdSec;
    }

    /** Target at {@code t} seconds since the trial started. */
    public double targetAt(double t) {
        if (t < settleSec) return low;
        int hold = (int) ((t - settleSec) / holdSec);
        if (hold >= 2 * cycles) return low;
        return hold % 2 == 0 ? high : low;
    }

    /** True while the motor should be cut for a virtual shot. */
    public boolean shotActive(double t) {
        if (shotSec <= 0 || t < settleSec) return false;
        int hold = (int) ((t - settleSec) / holdSec);
        double inHold = (t - settleSec) - hold * holdSec;
        return hold < 2 * cycles && inHold >= SHOT_AFTER_SEC && inHold < SHOT_AFTER_SEC + shotSec;
    }

    /** Time from trial start of the next virtual shot at or after {@code t}, or NaN if none. */
    public double nextShotAt(double t) {
        if (shotSec <= 0) return Double.NaN;
        for (int h = 0; h < 2 * cycles; h++) {
            double at = settleSec + h * holdSec + SHOT_AFTER_SEC;
            if (at >= t) return at;
        }
        return Double.NaN;
    }

    public boolean isDone(double t) {
        return t >= durationSec();
    }

    /** Feed every control-loop sample with its time since the trial started. */
    public void add(double t, double target, double velocity, double power) {
        // Start feeding just before the first step so the scorer sees the LOW→HIGH change.
        if (t >= settleSec - 0.1) score.add(t, target, velocity, power);
    }

    /**
     * Lower is better: Σ step IAE + Σ shot (recovery s + 0.05·over-speed %) + 20·chatter, plus a
     * penalty for steps still off target at the end (0 inside the ready band, growing smoothly
     * outside it: a hard +1 flipped on and off with noise) and 1 per shot not recovered.
     * Unlike {@link TuneScore#score()} it leaves out the step overshoot-% term: on small steps the
     * "overshoot" is the size of the encoder noise and made trials noisy. Real overshoot still
     * costs through IAE, which averages the noise out.
     */
    public double score() {
        double sc = 20 * score.chatter();
        for (StepAnalyzer.Result r : score.results) {
            if (r.kind == StepAnalyzer.Result.Kind.STEP) {
                if (r.to <= 0) continue;
                sc += r.iae;
                double band = Math.max(tolFrac * r.to, tolMin);
                sc += Math.max(0, r.ssRms / band - 1);
            } else {
                sc += (Double.isNaN(r.recoveryTime) ? 1 : r.recoveryTime) + 0.05 * r.dipOvershootPct;
            }
        }
        return sc;
    }

    /** Individual step / shot results of this trial (for display). */
    public List<StepAnalyzer.Result> results() {
        return score.results;
    }

    public double chatter() {
        return score.chatter();
    }
}
