package org.firstinspires.ftc.teamcode.subsystems.launcher;

import java.util.Arrays;

/**
 * Fits the launcher model to open-loop data (pure Java; used on the robot and in the simulator).
 *
 * Model: tau dv/dt = −v + kV w − kS, with w = power * battery / nominalVoltage, plus a dead time.
 *
 * Output-error fit: for each candidate dead time and tau, simulate the model over the whole log
 * and pick kV, kS by least squares (the simulated response is linear in them); golden-section
 * search on tau. Unlike a one-step regression (v[k+1] vs v[k]) this is not biased by encoder
 * noise, which matters for a flywheel because the noise is big next to the change per loop.
 * Samples near zero speed are skipped (friction is not linear there).
 */
public final class SysIdFit {

    public static final class Result {
        public boolean ok;
        public String message = "";
        public double tau, kV, kS, delaySec;
        /** RMS error of the simulated model vs the data, ticks/s (≈ encoder noise if the model fits). */
        public double rms;
        /** Fraction of velocity variance explained by the model (want &gt; 0.99). */
        public double r2;
        public int samples;
        /** Median logging period, s. */
        public double ts;

        @Override
        public String toString() {
            return ok ? String.format("tau=%.4f kV=%.1f kS=%.1f delay=%.3fs rms=%.1f r2=%.5f n=%d",
                    tau, kV, kS, delaySec, rms, r2, samples) : ("FAILED: " + message);
        }
    }

    /**
     * Open-loop power sequence used by the Launcher SysId OpMode (and the simulator's sysid check):
     * up and down steps of different sizes, each held until settled, then coast.
     */
    public static final double[] TEST_POWERS = {0.30, 0.60, 0.45, 0.90, 0.70, 1.00, 0.50, 0.80, 0.35, 0.0};

    private SysIdFit() {
    }

    /**
     * @param t           timestamps, s
     * @param v           measured velocity, ticks/s
     * @param power       power sent at each sample (after reading v at that sample)
     * @param battery     battery volts at each sample
     * @param nominalV    nominal voltage the model is normalised to
     * @param maxDelaySec largest dead time to try
     */
    public static Result fit(double[] t, double[] v, double[] power, double[] battery,
                             double nominalV, double maxDelaySec) {
        int n = t.length;
        Result fail = new Result();
        if (n < 50) {
            fail.message = "not enough samples (" + n + ")";
            return fail;
        }
        double[] dts = new double[n - 1];
        for (int i = 0; i < n - 1; i++) dts[i] = t[i + 1] - t[i];
        double[] sorted = dts.clone();
        Arrays.sort(sorted);
        double ts = sorted[sorted.length / 2];
        if (!(ts > 0)) {
            fail.message = "bad timestamps";
            return fail;
        }
        double vMax = 0;
        for (double x : v) vMax = Math.max(vMax, x);
        if (vMax < 50) {
            fail.message = "flywheel never moved forward — check motor names, directions and encoder cable";
            return fail;
        }
        double vMin = 0.08 * vMax;
        double[] w = new double[n];
        for (int i = 0; i < n; i++) w[i] = power[i] * battery[i] / nominalV;

        Result best = null;
        int maxNd = (int) Math.round(maxDelaySec / ts);
        double gr = (Math.sqrt(5) - 1) / 2;
        for (int nd = 0; nd <= maxNd; nd++) {
            // golden-section search on ln(tau) in [ln 0.02, ln 5]
            double lo = Math.log(0.02), hi = Math.log(5.0);
            double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
            Result r1 = evalOE(t, v, w, nd, Math.exp(x1), vMin);
            Result r2 = evalOE(t, v, w, nd, Math.exp(x2), vMin);
            for (int it = 0; it < 50; it++) {
                if (r1.rms < r2.rms) {
                    hi = x2; x2 = x1; r2 = r1;
                    x1 = hi - gr * (hi - lo);
                    r1 = evalOE(t, v, w, nd, Math.exp(x1), vMin);
                } else {
                    lo = x1; x1 = x2; r1 = r2;
                    x2 = lo + gr * (hi - lo);
                    r2 = evalOE(t, v, w, nd, Math.exp(x2), vMin);
                }
            }
            Result r = r1.rms < r2.rms ? r1 : r2;
            if (!r.ok) continue;
            r.ts = ts;
            r.delaySec = nd * ts;
            if (best == null || r.rms < best.rms) best = r;
        }
        if (best == null || best.kV <= 0) {
            fail.message = "fit failed (did the speed actually change? is the encoder on the right motor?)";
            return fail;
        }
        double sy = 0, syy = 0;
        int m = 0;
        for (int k = 0; k < n; k++) {
            if (v[k] < vMin) continue;
            sy += v[k];
            syy += v[k] * v[k];
            m++;
        }
        double var = syy / m - (sy / m) * (sy / m);
        best.r2 = var > 0 ? 1 - best.rms * best.rms / var : 0;
        return best;
    }

    /** Simulate basis responses for one tau / delay and solve for kV, kS. */
    private static Result evalOE(double[] t, double[] v, double[] w, int nd, double tau, double vMin) {
        int n = t.length;
        // vhat_k = x0_k + kV * xw_k + kS * xs_k
        //   x0: free decay of the initial speed, xw: response to w, xs: response to −1
        double x0 = v[0], xw = 0, xs = 0;
        double s11 = 0, s12 = 0, s22 = 0, b1 = 0, b2 = 0;
        int m = 0;
        double[] X0 = new double[n], XW = new double[n], XS = new double[n];
        for (int k = 0; k < n; k++) {
            X0[k] = x0;
            XW[k] = xw;
            XS[k] = xs;
            if (v[k] >= vMin) {
                double y = v[k] - x0;
                s11 += xw * xw;
                s12 += xw * xs;
                s22 += xs * xs;
                b1 += xw * y;
                b2 += xs * y;
                m++;
            }
            if (k == n - 1) break;
            double a = Math.exp(-(t[k + 1] - t[k]) / tau);
            double wk = k - nd >= 0 ? w[k - nd] : 0;
            x0 = a * x0;
            xw = a * xw + (1 - a) * wk;
            xs = a * xs - (1 - a);
        }
        Result r = new Result();
        double det = s11 * s22 - s12 * s12;
        if (m < 30 || Math.abs(det) < 1e-12) {
            r.rms = Double.POSITIVE_INFINITY;
            return r;
        }
        double kV = (b1 * s22 - b2 * s12) / det;
        double kS = (s11 * b2 - s12 * b1) / det;
        double ss = 0;
        for (int k = 0; k < n; k++) {
            if (v[k] < vMin) continue;
            double e = v[k] - (X0[k] + kV * XW[k] + kS * XS[k]);
            ss += e * e;
        }
        r.ok = true;
        r.tau = tau;
        r.kV = kV;
        r.kS = kS;
        r.samples = m;
        r.rms = Math.sqrt(ss / m);
        return r;
    }

    /** Straight-line fit of settled (w, v) points: v = kV w − kS. Returns {kV, kS, r2} or null. */
    public static double[] fitSteadyState(double[] w, double[] v) {
        int n = w.length;
        if (n < 2) return null;
        double sw = 0, sv = 0, sww = 0, swv = 0, svv = 0;
        for (int i = 0; i < n; i++) {
            sw += w[i];
            sv += v[i];
            sww += w[i] * w[i];
            swv += w[i] * v[i];
            svv += v[i] * v[i];
        }
        double den = n * sww - sw * sw;
        if (Math.abs(den) < 1e-12) return null;
        double slope = (n * swv - sw * sv) / den;
        double icpt = (sv - slope * sw) / n;
        double ssTot = svv - sv * sv / n, ssRes = 0;
        for (int i = 0; i < n; i++) {
            double e = v[i] - (slope * w[i] + icpt);
            ssRes += e * e;
        }
        return new double[]{slope, -icpt, ssTot > 0 ? 1 - ssRes / ssTot : 0};
    }
}
