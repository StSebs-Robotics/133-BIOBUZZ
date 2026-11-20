package org.firstinspires.ftc.teamcode.subsystems.launcher;

import java.util.ArrayList;
import java.util.List;

/**
 * Grades a run of the launcher with one number (lower is better), identically on the robot
 * (Launcher MPC Tuner) and in the desktop simulator, so sim results and robot results compare.
 *
 * score = Σ steps (IAE + 0.02·overshoot% + 0.01·ssRms [+1 if still off target at the end])
 *       + Σ dips (recovery seconds [1 if not recovered] + 0.05·over-speed% after recovering)
 *       + 20 · chatter
 *
 * chatter = RMS power change per loop while holding speed (motor "buzz" from chasing encoder
 * noise). 0.01/loop of chatter costs as much as 0.2 s of IAE. Coast-down steps (target 0) are
 * not scored — that is physics, not control.
 */
public final class TuneScore {

    private final StepAnalyzer analyzer;
    private final double tolFrac, tolMin;
    public final List<StepAnalyzer.Result> results = new ArrayList<>();

    private double lastTarget = Double.NaN, lastChangeT = 0, prevPower = Double.NaN;
    private boolean prevQuiet = false;
    private double chatterSs = 0;
    private int chatterN = 0;

    public TuneScore(double tolFrac, double tolMin) {
        this.tolFrac = tolFrac;
        this.tolMin = tolMin;
        analyzer = new StepAnalyzer(tolFrac, tolMin, 2.0);
    }

    /** Feed one control-loop sample. Returns a newly finished result (also kept in {@link #results}) or null. */
    public StepAnalyzer.Result add(double t, double target, double velocity, double power) {
        if (Double.isNaN(lastTarget) || target != lastTarget) lastChangeT = t;
        lastTarget = target;
        double band = Math.max(tolFrac * target, tolMin);
        boolean quiet = target > 0 && t - lastChangeT > 0.8 && Math.abs(velocity - target) < 3 * band;
        if (quiet && prevQuiet && !Double.isNaN(prevPower)) {
            double d = power - prevPower;
            chatterSs += d * d;
            chatterN++;
        }
        prevQuiet = quiet;
        prevPower = power;

        StepAnalyzer.Result r = analyzer.add(t, target, velocity);
        if (r != null) results.add(r);
        return r;
    }

    public double chatter() {
        return chatterN > 0 ? Math.sqrt(chatterSs / chatterN) : 0;
    }

    public double score() {
        double sc = 20 * chatter();
        for (StepAnalyzer.Result r : results) {
            if (r.kind == StepAnalyzer.Result.Kind.STEP) {
                if (r.to <= 0) continue;
                sc += r.iae + 0.02 * r.overshootPct + 0.01 * r.ssRms;
                // "Still off target" uses the RMS error over the end of the window, not the settle
                // time: settle time is decided by single noisy samples and made scores jumpy.
                if (r.ssRms > Math.max(tolFrac * r.to, tolMin)) sc += 1;
            } else {
                sc += (Double.isNaN(r.recoveryTime) ? 1 : r.recoveryTime) + 0.05 * r.dipOvershootPct;
            }
        }
        return sc;
    }
}
