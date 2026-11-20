package org.firstinspires.ftc.teamcode.subsystems.launcher;

/**
 * The launcher velocity MPC, as pure math (no FTC imports): the robot thread and the desktop
 * simulator run exactly this code.
 *
 * <h3>Model</h3>
 * First-order flywheel with battery compensation and an estimated load:
 * <pre>
 *   tau * dv/dt = -v + kV * u * (Vbatt / Vnom) - kS - load
 * </pre>
 * u = motor power, v = ticks/s. "load" is everything the model gets wrong — a ball going through,
 * friction changing as it warms up, a wrong kV. A Kalman filter estimates v and load from the
 * encoder every loop, so the MPC is offset-free without a hand-tuned integral term.
 *
 * <h3>Each loop</h3>
 * <ol>
 *   <li>{@link #observe}: Kalman update of (v, load) from the new encoder reading.</li>
 *   <li>Delay compensation: roll the estimate forward through the commands already sent but not
 *       yet visible in the encoder (Smith predictor), so the MPC plans from where the flywheel
 *       will be when this command actually lands.</li>
 *   <li>{@link #solve}: minimise over the next N power values
 *       <pre>  Σ q_j ((target − v_j)/kV)²  +  rDu Σ (u_i − u_{i−1})²   s.t. uMin ≤ u_i ≤ uMax</pre>
 *       a small box-constrained QP, solved with warm-started accelerated projected gradient
 *       (FISTA). Velocity error is divided by kV so the weights are dimensionless and do not need
 *       retuning when the gearing/encoder changes.</li>
 *   <li>{@link #commit}: remember the power actually sent (needed by the observer and the delay
 *       compensation).</li>
 * </ol>
 *
 * Not thread-safe; one instance belongs to one control thread.
 */
public final class MpcCore {

    private LauncherParams p = new LauncherParams().sanitized();

    // ---- observer state
    private double vHat = 0, loadHat = 0;
    private boolean initialized = false;
    private double kfGainV, kfGainL;      // steady-state Kalman gains

    // ---- input history (effective normalized input u*Vb/Vnom), oldest first.
    // hist[0] is the input that drove the interval that just ended; hist[1..nd] are in flight.
    private double[] hist = new double[1];
    private int delaySteps = 0;
    private double lastU = 0;

    // ---- QP workspace
    private int n = 0;
    private double[][] G, H;
    private double[] s, qw, f, u, uOld, y, g, plan;

    // ---- diagnostics from the last solve
    private double lastX0 = 0, lastCost = 0;
    private int lastIters = 0;

    public MpcCore() {
        setParams(p);
    }

    /** Apply new parameters. Keeps the estimate and input history where possible. */
    public void setParams(LauncherParams params) {
        this.p = params.sanitized();
        double ts = p.periodMs / 1000.0;
        int nd = (int) Math.round(p.delaySec / ts);
        if (nd != delaySteps || hist.length != nd + 1) {
            double[] h = new double[nd + 1];
            java.util.Arrays.fill(h, hist[hist.length - 1]);
            hist = h;
            delaySteps = nd;
        }
        if (p.horizonSteps != n) allocate(p.horizonSteps);
        computeKalmanGains(ts);
        if (learnCount == 0 && p.shotLeadSec > 0 && p.shotDropFrac > 0) { // learned on an earlier run
            learnedLead = p.shotLeadSec;
            learnedDrop = p.shotDropFrac;
            learnCount = LEARN_SHOTS;
        }
    }

    public LauncherParams getParams() {
        return p;
    }

    /** Forget the estimate (next {@link #observe} re-initialises from the measurement). */
    public void reset() {
        initialized = false;
        loadHat = 0;
        lastU = 0;
        java.util.Arrays.fill(hist, 0);
        if (plan != null) java.util.Arrays.fill(plan, 0);
    }

    // =============================================================================== observer

    /**
     * Kalman update with a new encoder velocity.
     *
     * @param measuredVel ticks/s from the encoder
     * @param dtSec       real time since the previous call (the thread jitters; the prediction uses it)
     */
    public void observe(double measuredVel, double dtSec) {
        observe(measuredVel, dtSec, false);
    }

    /**
     * @param armShotDetect true when a ball could be fired right now (holding speed near target).
     *                      Never arm during spin-up: model error there looks like a shot.
     */
    public void observe(double measuredVel, double dtSec, boolean armShotDetect) {
        if (!initialized) {
            vHat = measuredVel;
            loadHat = 0;
            initialized = true;
            return;
        }
        double dt = Math.max(1e-4, Math.min(dtSec, 0.1));
        tCore += dt;
        double a = Math.exp(-dt / p.tau);
        double vPred = predict(vHat, hist[0], loadHat, a);
        double innov = measuredVel - vPred;
        // 3-sample average of the encoder, for measuring how deep a shot's dip went
        avg3[avgIdx++ % 3] = measuredVel;
        double vAvg = (avg3[0] + avg3[1] + avg3[2]) / 3;

        // Shot detection (one-sided CUSUM on "slower than the model says"). A ball going through is
        // a sudden loss of speed, not a lasting load: without this the filter blames the drop on
        // load, the MPC keeps pushing after recovery and the next ball goes long.
        boolean wasShot = shotActive;
        if (p.shotDetectSigma > 0 && (armShotDetect || shotActive)) {
            double z = -innov / p.measNoise;           // > 0 when slower than predicted
            if (cusum == 0 && z > 1) {             // speed just started falling too fast
                loadBeforeCusum = loadHat;
                cusumStartT = tCore - dt;
                cusumStartV = vHat;
            }
            cusum = Math.max(0, cusum + z - 1);        // drift 1σ: robust if measNoise is a bit low
            if (!shotActive && cusum > p.shotDetectSigma) {
                shotActive = true;
                shotTime = 0;
                calmSamples = 0;
                shotCount++;
                loadHat = loadBeforeCusum;            // undo what the drop already did to the load
                shotMinV = vAvg;
                // a shot we were told about: measure how long the ball took from notifyShot()
                learningShot = notifyPending && cusumStartT - tNotify < NOTIFY_WINDOW_SEC;
                measuredLead = Math.max(0, cusumStartT - tNotify);
                notifyPending = false;
            }
            if (shotActive) {
                shotTime += dt;
                shotMinV = Math.min(shotMinV, vAvg);
                calmSamples = z < 1 ? calmSamples + 1 : 0;
                // ball gone once the speed stops falling; never stay "in a shot" for long, or a
                // real load change could never be learned
                if (calmSamples >= 2 || shotTime > MAX_SHOT_SEC) {
                    shotActive = false;
                    cusum = 0;
                    if (learningShot && cusumStartV > 0) {
                        learnShot(measuredLead, (cusumStartV - shotMinV) / cusumStartV);
                    }
                    learningShot = false;
                }
            }
        } else {
            cusum = 0;
        }
        // told about a shot that never came (jam, missed feed): forget it
        if (notifyPending && !shotActive && tCore - tNotify > NOTIFY_WINDOW_SEC) notifyPending = false;

        if (shotActive || wasShot) {
            vHat = measuredVel;                          // speed jumped: follow the encoder, keep load
        } else {
            vHat = vPred + kfGainV * innov;
            loadHat = loadHat + kfGainL * innov;
        }
        // Keep the load estimate sane: it can never need more than the motor can deliver.
        double lim = 2 * p.kV;
        loadHat = Math.max(-lim, Math.min(lim, loadHat));
    }

    private static final double MAX_SHOT_SEC = 0.2;
    private double cusum = 0, loadBeforeCusum = 0, shotTime = 0;
    private boolean shotActive = false;
    private int calmSamples = 0, shotCount = 0;

    // ---- shot feed-forward (notifyShot)
    /** A notified ball must arrive within this long, or it is forgotten (jam / missed feed). */
    private static final double NOTIFY_WINDOW_SEC = 1.0;
    private double tCore = 0, tNotify = 0, cusumStartT = 0, cusumStartV = 0, shotMinV = 0, measuredLead = 0;
    private boolean notifyPending = false, learningShot = false;
    private final double[] avg3 = new double[3];
    private int avgIdx = 0;
    private double learnedLead = 0, learnedDrop = 0;
    private int learnCount = 0;

    /**
     * The code just pushed a ball toward the wheel (call at the moment the feeder is commanded).
     * The first few notified shots only measure how long the ball takes to reach the wheel and how
     * much speed it costs; after {@link #LEARN_SHOTS} the MPC is told about each coming dip in
     * advance and starts the recovery early.
     *
     * @param secondsAgo how long ago the feeder was commanded (0 = now)
     */
    public void notifyShot(double secondsAgo) {
        tNotify = tCore - Math.max(0, secondsAgo);
        notifyPending = true;
    }

    /** Notified shots needed before the feed-forward is used. */
    public static final int LEARN_SHOTS = 2;

    private void learnShot(double lead, double drop) {
        drop = Math.max(0, Math.min(0.6, drop));
        lead = Math.max(0, Math.min(NOTIFY_WINDOW_SEC, lead));
        // average of the first few, then a moving average so it follows slow changes
        double alpha = learnCount < 4 ? 1.0 / (learnCount + 1) : 0.3;
        learnedLead += alpha * (lead - learnedLead);
        learnedDrop += alpha * (drop - learnedDrop);
        learnCount++;
    }

    /** Learned time from notifyShot() to the ball hitting the wheel, s (0 until learned). */
    public double getShotLead() { return learnedLead; }

    /** Learned speed lost per ball, fraction of speed (0 until learned). */
    public double getShotDrop() { return learnedDrop; }

    /** Notified shots measured so far (values from the params file count as {@link #LEARN_SHOTS}). */
    public int getShotLearnCount() { return learnCount; }

    /** A notified ball is on its way and has not hit the wheel yet. */
    public boolean isShotPending() { return notifyPending; }

    /** True once the feed-forward has enough to act on. */
    public boolean isShotFeedforwardReady() {
        return p.shotFeedforward > 0 && learnCount >= LEARN_SHOTS && learnedDrop > 0;
    }

    /** True while a ball is going through (speed dropping faster than the model allows). */
    public boolean isShotActive() { return shotActive; }

    /** Number of shots detected since this core was created. */
    public int getShotCount() { return shotCount; }

    /** One step of the model. Friction cannot spin the wheel backwards, so floor at 0 when coasting. */
    private double predict(double v, double wEff, double load, double a) {
        double next = a * v + (1 - a) * (p.kV * wEff - p.kS - load);
        if (next < 0 && v >= 0 && wEff >= 0) next = 0;
        return next;
    }

    /**
     * Steady-state Kalman gains for x = [v, load], with
     *   v+    = a v − (1−a) load + (1−a) kV w − (1−a) kS
     *   load+ = load
     * Q = diag(velNoise², loadNoise²), R = measNoise². Iterated Riccati; 2×2 so it is cheap.
     */
    private void computeKalmanGains(double ts) {
        double a = Math.exp(-ts / p.tau);
        double a12 = -(1 - a);
        double q1 = p.velNoise * p.velNoise, q2 = p.loadNoise * p.loadNoise, r = p.measNoise * p.measNoise;
        double p11 = r, p12 = 0, p22 = r; // initial covariance
        double k1 = 0, k2 = 0;
        for (int i = 0; i < 2000; i++) {
            // predict: P- = A P A' + Q, A = [[a, a12],[0, 1]]
            double m11 = a * a * p11 + 2 * a * a12 * p12 + a12 * a12 * p22 + q1;
            double m12 = a * p12 + a12 * p22;
            double m22 = p22 + q2;
            // update with C = [1, 0]
            double sInv = 1.0 / (m11 + r);
            double nk1 = m11 * sInv, nk2 = m12 * sInv;
            p11 = (1 - nk1) * m11;
            p12 = (1 - nk1) * m12;
            p22 = m22 - nk2 * m12;
            boolean done = Math.abs(nk1 - k1) < 1e-12 && Math.abs(nk2 - k2) < 1e-12;
            k1 = nk1;
            k2 = nk2;
            if (done) break;
        }
        kfGainV = k1;
        kfGainL = k2;
    }

    // =============================================================================== MPC

    /**
     * Solve the MPC and return the power to send now. Call after {@link #observe}, then call
     * {@link #commit} with whatever you actually sent.
     *
     * @param target   ticks/s
     * @param battery  volts (latest reading)
     */
    public double solve(double target, double battery) {
        double vb = Math.max(battery, 6.0);
        double ts = p.periodMs / 1000.0;

        // 1. Delay compensation: where will v be when this command takes effect?
        double aCtl = Math.exp(-ts / p.tau);
        double x0 = vHat;
        for (int i = 1; i <= delaySteps; i++) x0 = predict(x0, hist[i], loadHat, aCtl);
        lastX0 = x0;

        // 2. Build the condensed QP over N prediction steps of length tp.
        double tp = p.horizonSec / n;
        double a = Math.exp(-tp / p.tau);
        double b = (1 - a) * p.kV * vb / p.nominalVoltage; // ticks/s per unit power per step
        double c = -(1 - a) * (p.kS + loadHat);
        double invKv2 = 1.0 / (p.kV * p.kV);

        // Shot feed-forward. dipAt = when the notified ball is expected to hit, relative to the
        // moment this command takes effect (x0 is already delaySteps ahead of now).
        // Only used once the command lands at or after contact (dipAt < one period): planning for
        // the dip any earlier just spins the wheel up before the ball arrives, and that ball then
        // leaves too fast. The physics allows nothing better: a ball takes ~10% of the speed in
        // ~30 ms, far faster than the motor can add it back. What this does buy is starting the
        // full recovery push the moment the ball hits, instead of 20–30 ms later when the encoder
        // shows the drop. Skipped once the shot is actually seen (shotActive).
        double dipAt = 0, dipSize = 0;
        if (notifyPending && !shotActive && isShotFeedforwardReady()) {
            dipAt = tNotify + learnedLead - tCore - delaySteps * ts;
            if (dipAt > -0.05 && dipAt < ts) dipSize = p.shotFeedforward * learnedDrop * target;
        }
        // free response s_j (no power), and Toeplitz impulse response G[j][i] = a^(j-i) b
        double sj = x0;
        for (int j = 0; j < n; j++) {
            sj = a * sj + c;
            s[j] = sj;
            if (dipSize > 0 && (j + 1) * tp >= dipAt) {
                // known ball coming: the speed will be dipSize lower from dipAt on, decaying like
                // any other speed error; the MPC plans the power to cover it
                s[j] -= dipSize * Math.pow(a, ((j + 1) * tp - Math.max(dipAt, 0)) / tp);
            }
            qw[j] = p.qVel + (j == n - 1 ? p.qTerminal : 0);
        }
        double[] imp = y; // reuse as scratch for impulse response a^k b
        imp[0] = b;
        for (int k = 1; k < n; k++) imp[k] = imp[k - 1] * a;
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) G[j][i] = i <= j ? imp[j - i] : 0;
        }

        // H = 2/kV² Gᵀ diag(q) G + 2 rDu DᵀD ;  f = 2/kV² Gᵀ diag(q) (s − r) − 2 rDu e0 lastU
        for (int i = 0; i < n; i++) {
            for (int k = i; k < n; k++) {
                double acc = 0;
                for (int j = k; j < n; j++) acc += qw[j] * G[j][i] * G[j][k]; // G[j][*]=0 for j<max(i,k)
                acc *= 2 * invKv2;
                H[i][k] = acc;
                H[k][i] = acc;
            }
        }
        double r2 = 2 * p.rDu;
        for (int i = 0; i < n; i++) {
            H[i][i] += r2 * (i == n - 1 ? 1 : 2);
            if (i + 1 < n) {
                H[i][i + 1] -= r2;
                H[i + 1][i] -= r2;
            }
        }
        for (int i = 0; i < n; i++) {
            double acc = 0;
            for (int j = i; j < n; j++) acc += G[j][i] * qw[j] * (s[j] - target);
            f[i] = 2 * invKv2 * acc;
        }
        f[0] -= r2 * lastU;

        // 3. FISTA with adaptive restart. Lipschitz bound = max abs row sum of H (Gershgorin).
        double lip = 0;
        for (int i = 0; i < n; i++) {
            double row = 0;
            for (int k = 0; k < n; k++) row += Math.abs(H[i][k]);
            lip = Math.max(lip, row);
        }
        double step = 1.0 / lip;

        // warm start: previous plan shifted by one control period (≈ unchanged when tp >> ts)
        for (int i = 0; i < n; i++) u[i] = clip(plan[i]);
        System.arraycopy(u, 0, y, 0, n);
        double t = 1;
        int it = 0;
        for (; it < p.maxIters; it++) {
            // g = H y + f
            for (int i = 0; i < n; i++) {
                double acc = f[i];
                double[] hi = H[i];
                for (int k = 0; k < n; k++) acc += hi[k] * y[k];
                g[i] = acc;
            }
            System.arraycopy(u, 0, uOld, 0, n);
            double maxChange = 0, restartTest = 0;
            for (int i = 0; i < n; i++) {
                u[i] = clip(y[i] - step * g[i]);
                double d = u[i] - uOld[i];
                maxChange = Math.max(maxChange, Math.abs(d));
                restartTest += g[i] * d;
            }
            if (maxChange < 1e-7) {
                it++;
                break;
            }
            double tNew;
            if (restartTest > 0) { // gradient-based restart (O'Donoghue & Candès)
                tNew = 1;
                System.arraycopy(u, 0, y, 0, n);
            } else {
                tNew = 0.5 * (1 + Math.sqrt(1 + 4 * t * t));
                double mom = (t - 1) / tNew;
                for (int i = 0; i < n; i++) y[i] = u[i] + mom * (u[i] - uOld[i]);
            }
            t = tNew;
        }
        lastIters = it;
        System.arraycopy(u, 0, plan, 0, n);

        // cost for diagnostics: ½uᵀHu + fᵀu (+ const)
        double cost = 0;
        for (int i = 0; i < n; i++) {
            double hu = 0;
            for (int k = 0; k < n; k++) hu += H[i][k] * u[k];
            cost += 0.5 * u[i] * hu + f[i] * u[i];
        }
        lastCost = cost;

        double out = u[0];
        if (p.maxDuPerSec > 0) {
            double lim = p.maxDuPerSec * ts;
            out = Math.max(lastU - lim, Math.min(lastU + lim, out));
        }
        return clip(out);
    }

    /** Record the power that was actually sent this loop (MPC output, open-loop, or 0). */
    public void commit(double powerSent, double battery) {
        double vb = Math.max(battery, 6.0);
        System.arraycopy(hist, 1, hist, 0, hist.length - 1);
        hist[hist.length - 1] = powerSent * vb / p.nominalVoltage;
        lastU = powerSent;
    }

    /** Clear the warm start (e.g. when the motor was off) so the solver starts from the current power. */
    public void resetPlan(double fill) {
        java.util.Arrays.fill(plan, clip(fill));
    }

    /** Steady-state power for a target at this battery voltage, including the current load estimate. */
    public double feedforwardPower(double target, double battery) {
        double vb = Math.max(battery, 6.0);
        return (target + p.kS + loadHat) / p.kV * p.nominalVoltage / vb;
    }

    // =============================================================================== getters

    public double getVelocityEstimate() { return vHat; }
    public double getLoadEstimate() { return loadHat; }
    /** Predicted velocity at the moment the current command lands (after delay compensation). */
    public double getPredictedVelocity() { return lastX0; }
    public double getLastCost() { return lastCost; }
    public int getLastIterations() { return lastIters; }
    public int getDelaySteps() { return delaySteps; }
    public double getKalmanGainVel() { return kfGainV; }
    public double getKalmanGainLoad() { return kfGainL; }
    /** The current optimal power plan over the horizon (do not modify). */
    public double[] getPlan() { return plan; }

    // =============================================================================== helpers

    private double clip(double v) {
        return Math.max(p.uMin, Math.min(p.uMax, v));
    }

    private void allocate(int size) {
        double[] oldPlan = plan;
        n = size;
        G = new double[n][n];
        H = new double[n][n];
        s = new double[n];
        qw = new double[n];
        f = new double[n];
        u = new double[n];
        uOld = new double[n];
        y = new double[n];
        g = new double[n];
        plan = new double[n];
        if (oldPlan != null) java.util.Arrays.fill(plan, oldPlan[0]);
    }
}
