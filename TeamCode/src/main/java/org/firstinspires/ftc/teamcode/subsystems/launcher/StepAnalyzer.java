package org.firstinspires.ftc.teamcode.subsystems.launcher;

/**
 * Scores the launcher's response so tuning is by numbers, not by ear. Feed it every sample; it
 * returns a {@link Result} when a step response (target changed) or a dip recovery (a ball went
 * through while at speed) is finished. Pure Java; used by the tuner OpMode and the simulator.
 * The velocity is smoothed with a 20 ms filter first, so the numbers describe the flywheel, not
 * the encoder noise (this adds ~20 ms to the reported rise time).
 */
public final class StepAnalyzer {

    public static final class Result {
        public enum Kind { STEP, DIP }

        public Kind kind;
        public double from, to;
        /** 10→90% rise time, s (NaN if it never got to 90%). */
        public double riseTime = Double.NaN;
        /** Peak beyond the target, % of the step size. */
        public double overshootPct;
        /** Time until it entered the tolerance band for good, s (NaN = never settled in window). */
        public double settleTime = Double.NaN;
        /** RMS error over the last quarter of the window, ticks/s. */
        public double ssRms;
        /** ∫|error| dt / |step|, seconds. One number to minimise: lower is better. */
        public double iae;
        /** DIP only: how far below target it fell, ticks/s. */
        public double dipDepth;
        /** DIP only: time from leaving the band to being back in it, s. */
        public double recoveryTime = Double.NaN;
        /** DIP only: worst over-speed in the 0.6 s after recovering, % of target (next ball goes long). */
        public double dipOvershootPct;

        public String summary() {
            if (kind == Kind.DIP) {
                return String.format("DIP  -%.0f t/s  recover %s  then OS %.1f%%", dipDepth, fmt(recoveryTime), dipOvershootPct);
            }
            return String.format("STEP %.0f→%.0f  rise %s  OS %.1f%%  settle %s  ss %.1f  IAE %.3f",
                    from, to, fmt(riseTime), overshootPct, fmt(settleTime), ssRms, iae);
        }

        private static String fmt(double s) {
            return Double.isNaN(s) ? "--" : String.format("%.0fms", s * 1000);
        }
    }

    private final double tolFrac, tolMin, window;
    private static final int MAX = 4000;
    private final double[] ts = new double[MAX], vs = new double[MAX];
    private int count = 0;

    private boolean inStep = false;
    private double t0, from, to, lastTarget = Double.NaN, lastV = 0;

    private boolean inDip = false, inPost = false;
    private double dipStart, dipMin, postEnd, postMax;
    private Result pending;
    private static final double POST_SEC = 0.6;
    private boolean settledAtTarget = false;

    /** Light smoothing so encoder noise does not dominate the settle/overshoot numbers. */
    private static final double SMOOTH_SEC = 0.02;
    private double vf = Double.NaN, lastT = Double.NaN;

    /**
     * @param tolFrac   settle band as a fraction of target (e.g. 0.02)
     * @param tolMin    minimum settle band, ticks/s
     * @param windowSec how long after a target change to watch before scoring
     */
    public StepAnalyzer(double tolFrac, double tolMin, double windowSec) {
        this.tolFrac = tolFrac;
        this.tolMin = tolMin;
        this.window = windowSec;
    }

    private double tol(double target) {
        return Math.max(tolFrac * Math.abs(target), tolMin);
    }

    /** @return a finished result, or null. */
    public Result add(double t, double target, double vRaw) {
        Result out = null;
        if (Double.isNaN(vf)) {
            vf = vRaw;
        } else {
            double alpha = 1 - Math.exp(-Math.max(0, t - lastT) / SMOOTH_SEC);
            vf += alpha * (vRaw - vf);
        }
        lastT = t;
        double v = vf;
        if (Double.isNaN(lastTarget)) lastTarget = target;

        if (Math.abs(target - lastTarget) > 1e-6) {
            if (inPost) out = finishPost();
            if (inStep) out = finishStep(); // interrupted: score what we have
            inStep = true;
            inDip = false;
            settledAtTarget = false;
            t0 = t;
            from = lastV;
            to = target;
            count = 0;
        }
        lastTarget = target;
        lastV = v;

        if (inStep) {
            if (count < MAX) {
                ts[count] = t;
                vs[count] = v;
                count++;
            }
            if (t - t0 >= window) {
                out = finishStep();
                inStep = false;
                settledAtTarget = Math.abs(v - target) < tol(target);
            }
            return out;
        }

        // Dip detection while holding a non-zero target.
        if (target <= 0) return out;
        double band = tol(target);
        if (inPost) {
            postMax = Math.max(postMax, v);
            boolean newDip = v < target - Math.max(2 * band, 0.03 * target);
            if (t >= postEnd || newDip) out = finishPost();
            if (!newDip) return out;
        }
        if (!inDip) {
            if (Math.abs(v - target) < band) settledAtTarget = true;
            if (settledAtTarget && v < target - Math.max(2 * band, 0.03 * target)) {
                inDip = true;
                dipStart = t;
                dipMin = v;
            }
        } else {
            dipMin = Math.min(dipMin, v);
            if (Math.abs(v - target) < band) {
                Result r = new Result();
                r.kind = Result.Kind.DIP;
                r.from = r.to = target;
                r.dipDepth = target - dipMin;
                r.recoveryTime = t - dipStart;
                inDip = false;
                inPost = true;      // keep watching for over-speed before reporting
                postEnd = t + POST_SEC;
                postMax = v;
                pending = r;
            } else if (t - dipStart > window) {
                Result r = new Result();
                r.kind = Result.Kind.DIP;
                r.from = r.to = target;
                r.dipDepth = target - dipMin;
                inDip = false;
                settledAtTarget = false;
                out = r;
            }
        }
        return out;
    }

    private Result finishPost() {
        inPost = false;
        Result r = pending;
        pending = null;
        if (r != null) r.dipOvershootPct = Math.max(0, 100 * (postMax - r.to) / r.to);
        return r;
    }

    private Result finishStep() {
        Result r = new Result();
        r.kind = Result.Kind.STEP;
        r.from = from;
        r.to = to;
        double delta = to - from;
        if (count < 3 || Math.abs(delta) < 1) return null;
        double band = tol(to);
        double t10 = Double.NaN, t90 = Double.NaN, peak = 0, iae = 0;
        int lastOutside = -1;
        for (int i = 0; i < count; i++) {
            double frac = (vs[i] - from) / delta;
            if (Double.isNaN(t10) && frac >= 0.1) t10 = ts[i];
            if (Double.isNaN(t90) && frac >= 0.9) t90 = ts[i];
            peak = Math.max(peak, frac - 1);
            if (Math.abs(vs[i] - to) >= band) lastOutside = i;
            if (i > 0) iae += Math.abs(to - vs[i]) * (ts[i] - ts[i - 1]);
        }
        r.riseTime = t90 - t10;
        r.overshootPct = 100 * peak;
        if (lastOutside < count - 1) r.settleTime = (lastOutside < 0 ? ts[0] : ts[lastOutside + 1]) - t0;
        r.iae = iae / Math.abs(delta);
        int startSs = (int) (count * 0.75);
        double ss = 0;
        for (int i = startSs; i < count; i++) ss += (vs[i] - to) * (vs[i] - to);
        r.ssRms = Math.sqrt(ss / Math.max(1, count - startSs));
        return r;
    }
}
