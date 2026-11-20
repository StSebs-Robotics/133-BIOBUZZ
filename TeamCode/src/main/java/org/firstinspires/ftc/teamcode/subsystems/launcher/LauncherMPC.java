package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.CachedMotor;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.locks.LockSupport;

/**
 * Launcher flywheel velocity controller: MPC + Kalman load observer, running on its own thread
 * at a fixed rate (default 100 Hz) so it is not slowed down by the OpMode loop, vision, pathing...
 *
 * <pre>
 *   LauncherMPC launcher = new LauncherMPC(this);      // in init; thread starts, motor stays off
 *   launcher.setTargetVelocity(1800);                  // ticks/s — that's it, no update() calls
 *   if (launcher.isReady()) feedBall();
 *   launcher.off();                                    // coast
 * </pre>
 *
 * Safety: the thread stops itself (and zeroes the motors) within ~50 ms of the OpMode that created
 * it ending, even if you forget {@link #close()}.
 *
 * Tuning is stored in /sdcard/FIRST/launcher/&lt;name&gt;.properties and loaded here. Use the
 * "Launcher SysId" and "Launcher MPC Tuner" OpModes (tuning group) to create it — see
 * TeamCode/.../subsystems/launcher/README.md.
 *
 * Hardware access from a second thread: motor writes are synchronized by the SDK. Velocity is read
 * with its own bulk read on the launcher's hub ({@link LynxModule#getBulkData()}), which also
 * refreshes that hub's bulk cache, so code on the main thread using MANUAL caching just sees slightly
 * fresher data — nothing breaks.
 */
public class LauncherMPC {

    // =============================================================================================
    // Hardware configuration — names must match the Driver Station robot configuration.
    // =============================================================================================

    public static class Config {
        public final List<String> motorNames = new ArrayList<>();
        public final List<DcMotorSimple.Direction> directions = new ArrayList<>();
        /** Index (into the motors added) of the motor whose encoder is plugged in. */
        public int encoderIndex = 0;
        /** Params file: /sdcard/FIRST/launcher/&lt;name&gt;.properties */
        public String name = "launcher";

        /** Add a motor. Directions must make positive power spin the flywheel the shooting way. */
        public Config motor(String hardwareName, DcMotorSimple.Direction direction) {
            motorNames.add(hardwareName);
            directions.add(direction);
            return this;
        }

        public Config encoder(int index) {
            encoderIndex = index;
            return this;
        }

        public Config name(String fileName) {
            name = fileName;
            return this;
        }
    }

    /** THE launcher. Edit this when the hardware changes. */
    public static Config defaultConfig() {
        return new Config()
                // Direction not checked on the robot yet: run Launcher SysId; if it reports
                // "flywheel never moved forward", change this to REVERSE.
                .motor("motor3", DcMotorSimple.Direction.FORWARD)
                // .motor("launcher2", DcMotorSimple.Direction.REVERSE)   // second flywheel motor
                .encoder(0)
                .name("launcher");
    }

    public static final File PARAMS_DIR = new File(AppUtil.FIRST_FOLDER, "launcher");

    // =============================================================================================

    public enum Mode { OFF, VELOCITY, OPEN_LOOP }

    /** Immutable snapshot published by the control thread every loop. */
    public static final class State {
        public final double time, target, velocity, estimate, load, power, battery;
        public final double loopMs, solveMs;
        public final int iterations;
        public final boolean ready;
        public final Mode mode;
        /** A ball is going through right now (speed dropping faster than the model allows). */
        public final boolean shotActive;
        /** Shots detected since the launcher was created. */
        public final int shotCount;

        State(double time, Mode mode, double target, double velocity, double estimate, double load,
              double power, double battery, double loopMs, double solveMs, int iterations, boolean ready,
              boolean shotActive, int shotCount) {
            this.shotActive = shotActive;
            this.shotCount = shotCount;
            this.time = time;
            this.mode = mode;
            this.target = target;
            this.velocity = velocity;
            this.estimate = estimate;
            this.load = load;
            this.power = power;
            this.battery = battery;
            this.loopMs = loopMs;
            this.solveMs = solveMs;
            this.iterations = iterations;
            this.ready = ready;
        }
    }

    private static final String[] LOG_COLUMNS =
            {"t", "target", "velocity", "estimate", "load", "power", "battery", "loopMs", "solveMs"};

    // ---- hardware
    private final OpMode owner;
    private final Config config;
    private final DcMotorEx[] motors;
    private final CachedMotor[] cached;
    private final DcMotorEx encoderMotor;
    private final LynxModule hub;          // null → fall back to encoderMotor.getVelocity()
    private final int encoderPort;
    private final double encoderSign;
    private final File paramsFile;

    // ---- shared with the control thread
    private volatile Mode mode = Mode.OFF;
    private volatile double target = 0;
    private volatile double openLoopPower = 0;
    private volatile LauncherParams params;
    private volatile LauncherParams pendingParams;
    private volatile State state;
    private volatile boolean running = true;
    private volatile Throwable error;
    private volatile boolean resetObserver = false;
    private volatile long virtualShotUntilNs = 0;
    private volatile long notifyNs = 0;
    // learned shot timing, published by the control thread
    private volatile double shotLead = 0, shotDrop = 0;
    private volatile int shotLearnCount = 0;

    // ---- log ring buffer (guarded by logLock)
    private final Object logLock = new Object();
    private float[][] log = new float[0][];
    private int logHead = 0, logCount = 0;
    private volatile boolean logging = false;

    private final Thread thread;
    private final long startNs = System.nanoTime();

    public LauncherMPC(OpMode opMode) {
        this(opMode, defaultConfig());
    }

    public LauncherMPC(OpMode opMode, Config config) {
        this.owner = opMode;
        this.config = config;
        HardwareMap hw = opMode.hardwareMap;
        int n = config.motorNames.size();
        if (n == 0) throw new IllegalArgumentException("LauncherMPC: no motors configured");
        motors = new DcMotorEx[n];
        cached = new CachedMotor[n];
        CachedMotor.Params cp = new CachedMotor.Params();
        cp.writeThreshold = 0.002; // ~0.2% power; voltage comp is done by the MPC model itself
        for (int i = 0; i < n; i++) {
            DcMotorEx m = hw.get(DcMotorEx.class, config.motorNames.get(i));
            m.setDirection(config.directions.get(i));
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our controller, not the hub's PIDF
            m.setPower(0);
            motors[i] = m;
            cached[i] = new CachedMotor(m, cp);
        }
        encoderMotor = motors[Math.max(0, Math.min(config.encoderIndex, n - 1))];
        encoderPort = encoderMotor.getPortNumber();
        hub = findHub(encoderMotor);
        // Raw bulk data is in motor-native direction; DcMotorImpl flips it by the operational direction.
        DcMotorSimple.Direction op = encoderMotor.getDirection();
        if (encoderMotor.getMotorType().getOrientation() == Rotation.CCW) op = op.inverted();
        encoderSign = op == DcMotorSimple.Direction.REVERSE ? -1 : 1;

        paramsFile = LauncherParams.fileFor(PARAMS_DIR, config.name);
        params = LauncherParams.loadOrDefault(paramsFile).sanitized();

        thread = new Thread(this::run, "LauncherMPC-" + config.name);
        thread.setDaemon(true);
        thread.setPriority(Thread.MAX_PRIORITY - 1);
        thread.start();
    }

    // =============================================================================================
    // Commands (call from anywhere, any time)
    // =============================================================================================

    /** Spin to this speed (ticks/s) and hold it. 0 or less = coast. */
    public void setTargetVelocity(double ticksPerSec) {
        target = ticksPerSec;
        mode = Mode.VELOCITY;
    }

    /** Same as {@link #setTargetVelocity} in flywheel RPM (uses params.ticksPerRev). */
    public void setTargetRpm(double rpm) {
        setTargetVelocity(rpm * params.ticksPerRev / 60.0);
    }

    /** Raw power, no control (used by SysId). The observer keeps running. */
    public void setOpenLoopPower(double power) {
        openLoopPower = power;
        mode = Mode.OPEN_LOOP;
    }

    /** Motor off, coasting. */
    public void off() {
        target = 0;
        mode = Mode.OFF;
    }

    /** Stop the thread and the motors. Safe to call twice. Also happens automatically at OpMode end. */
    public void close() {
        running = false;
        thread.interrupt();
        try {
            thread.join(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        for (CachedMotor m : cached) safeZero(m);
    }

    // =============================================================================================
    // Status
    // =============================================================================================

    /** Latest snapshot (never null after the first loop; check {@link #getError()} if it stays null). */
    public State getState() {
        return state;
    }

    /** At speed (within tolerance for readySettleMs). */
    public boolean isReady() {
        State s = state;
        return s != null && s.ready;
    }

    /**
     * Optional: call at the moment your code pushes a ball toward the wheel (when you command the
     * feeder). You don't need to know how long the ball takes to arrive: the first
     * {@link MpcCore#LEARN_SHOTS} notified balls are timed and measured, and from then on the
     * controller starts recovering just before each ball hits, so the dip is smaller and shorter.
     * The learned timing is saved with the params (saveParams / the tuners), so it is learned once.
     *
     * Call it once per ball. If a ball never arrives (jam), it is forgotten after 1 s.
     */
    public void notifyShot() {
        notifyNs = System.nanoTime();
    }

    /** Learned time from notifyShot() to the ball hitting the wheel, s (0 = not learned yet). */
    public double getShotLeadSec() {
        return shotLead;
    }

    /** Learned speed lost per ball, fraction of speed (0 = not learned yet). */
    public double getShotDropFrac() {
        return shotDrop;
    }

    /** Notified balls measured so far; the feed-forward starts at {@link MpcCore#LEARN_SHOTS}. */
    public int getShotLearnCount() {
        return shotLearnCount;
    }

    /** Balls detected going through the launcher (from the speed dips) since creation. */
    public int getShotCount() {
        State s = state;
        return s == null ? 0 : s.shotCount;
    }

    /** Measured velocity, ticks/s. */
    public double getVelocity() {
        State s = state;
        return s == null ? 0 : s.velocity;
    }

    public double getTarget() {
        return mode == Mode.VELOCITY ? target : 0;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isRunning() {
        return thread.isAlive();
    }

    /** Exception that killed the control thread, or null. */
    public Throwable getError() {
        return error;
    }

    /** False if the hub could not be found and velocity falls back to motor.getVelocity() (may be stale with MANUAL bulk caching). */
    public boolean hasDirectBulkRead() {
        return hub != null;
    }

    // =============================================================================================
    // Parameters
    // =============================================================================================

    /**
     * A copy of the active parameters; change it and pass it to {@link #setParams}. Includes the
     * learned notifyShot() timing, so saving it keeps what was learned.
     */
    public LauncherParams getParams() {
        LauncherParams p = pendingParams;
        LauncherParams c = (p != null ? p : params).copy();
        if (shotLearnCount > 0 && shotDrop > 0) {
            c.shotLeadSec = shotLead;
            c.shotDropFrac = shotDrop;
        }
        return c;
    }

    /** Applied by the control thread at the start of its next loop. */
    public void setParams(LauncherParams p) {
        pendingParams = p.sanitized();
    }

    public File getParamsFile() {
        return paramsFile;
    }

    public void saveParams(String comment) throws IOException {
        getParams().save(paramsFile, comment);
    }

    /**
     * Test tool (used by Launcher Auto-Tune): cut the motor for {@code seconds} without telling the
     * controller, so it sees a speed loss it cannot explain — the same thing a ball does.
     */
    public void virtualShot(double seconds) {
        virtualShotUntilNs = System.nanoTime() + (long) (seconds * 1e9);
    }

    /** Re-initialise the Kalman estimate from the next measurement (e.g. after changing the model). */
    public void resetObserver() {
        resetObserver = true;
    }

    // =============================================================================================
    // Logging
    // =============================================================================================

    /** Start recording every control loop into a ring buffer (capacity in samples; 100 Hz → 6000 = 60 s). */
    public void startLogging(int capacity) {
        synchronized (logLock) {
            log = new float[capacity][];
            logHead = 0;
            logCount = 0;
        }
        logging = true;
    }

    public void stopLogging() {
        logging = false;
    }

    /** Copy of the log, oldest first; columns as {@link #logColumns()}. */
    public double[][] getLog() {
        synchronized (logLock) {
            double[][] out = new double[logCount][];
            int start = (logHead - logCount + log.length) % Math.max(log.length, 1);
            for (int i = 0; i < logCount; i++) {
                float[] r = log[(start + i) % log.length];
                double[] d = new double[r.length];
                for (int k = 0; k < r.length; k++) d[k] = r[k];
                out[i] = d;
            }
            return out;
        }
    }

    /** Log rows newer than time {@code sinceT} (column 0), oldest first. Cheap to call every loop. */
    public double[][] getLogSince(double sinceT) {
        synchronized (logLock) {
            int n = 0;
            while (n < logCount) {
                float[] r = log[(logHead - 1 - n + log.length) % log.length];
                if (r[0] <= sinceT) break;
                n++;
            }
            double[][] out = new double[n][];
            for (int i = 0; i < n; i++) {
                float[] r = log[(logHead - n + i + log.length) % log.length];
                double[] d = new double[r.length];
                for (int k = 0; k < r.length; k++) d[k] = r[k];
                out[i] = d;
            }
            return out;
        }
    }

    public static String[] logColumns() {
        return LOG_COLUMNS.clone();
    }

    /** Write the log to /sdcard/FIRST/launcher/&lt;prefix&gt;-&lt;time&gt;.csv; returns the file. */
    public File dumpLog(String prefix) throws IOException {
        double[][] rows = getLog();
        if (!PARAMS_DIR.exists() && !PARAMS_DIR.mkdirs()) throw new IOException("cannot create " + PARAMS_DIR);
        String stamp = new java.text.SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new java.util.Date());
        File f = new File(PARAMS_DIR, prefix + "-" + stamp + ".csv");
        try (PrintWriter pw = new PrintWriter(f)) {
            pw.println(String.join(",", LOG_COLUMNS));
            for (double[] r : rows) {
                StringBuilder sb = new StringBuilder();
                for (int k = 0; k < r.length; k++) {
                    if (k > 0) sb.append(',');
                    sb.append(String.format(Locale.US, k == 0 ? "%.4f" : "%.3f", r[k]));
                }
                pw.println(sb);
            }
        }
        return f;
    }

    // =============================================================================================
    // Control thread
    // =============================================================================================

    private void run() {
        MpcCore core = new MpcCore();
        core.setParams(params);
        long periodNs = params.periodMs * 1_000_000L;
        long next = System.nanoTime();
        long last = next;
        long lastVoltNs = 0, lastOwnerCheckNs = 0;
        double battery = readBattery(12.0);
        double lastU = 0, readySinceS = -1;
        Mode lastMode = Mode.OFF;
        try {
            while (running && !Thread.currentThread().isInterrupted()) {
                long now = System.nanoTime();
                if (now - lastOwnerCheckNs > 50_000_000L) {
                    lastOwnerCheckNs = now;
                    if (!ownerStillActive()) break;
                }

                LauncherParams np = pendingParams;
                if (np != null) {
                    pendingParams = null;
                    params = np;
                    core.setParams(np);
                    periodNs = np.periodMs * 1_000_000L;
                }
                if (resetObserver) {
                    resetObserver = false;
                    core.reset();
                }
                LauncherParams p = params;

                double dt = (now - last) / 1e9;
                last = now;
                double vel = readVelocity();
                if (now - lastVoltNs > 100_000_000L) { // battery is a separate transaction: 10 Hz
                    lastVoltNs = now;
                    double b = readBattery(battery);
                    battery = battery + 0.3 * (b - battery);
                }
                Mode m = mode;
                double tgt = target;
                // Shots only happen while holding speed; arming during spin-up would mistake model
                // error for balls. Armed within 3× the ready band of the target.
                boolean arm = m == Mode.VELOCITY && tgt > 0
                        && Math.abs(core.getVelocityEstimate() - tgt) < 3 * Math.max(p.readyTolFrac * tgt, p.readyTolMin);
                core.observe(vel, dt, arm);
                long nn = notifyNs;
                if (nn != 0) {
                    notifyNs = 0;
                    core.notifyShot(Math.max(0, now - nn) / 1e9);
                }
                shotLead = core.getShotLead();
                shotDrop = core.getShotDrop();
                shotLearnCount = core.getShotLearnCount();

                double u;
                long solveStart = System.nanoTime();
                if (m == Mode.VELOCITY && tgt > 0) {
                    if (lastMode != Mode.VELOCITY) core.resetPlan(lastU);
                    u = core.solve(tgt, battery);
                } else if (m == Mode.OPEN_LOOP) {
                    u = Math.max(-1, Math.min(1, openLoopPower));
                    core.resetPlan(u);
                } else {
                    u = 0;
                    core.resetPlan(0);
                }
                double solveMs = (System.nanoTime() - solveStart) / 1e6;
                lastMode = m;

                // Virtual shot (auto-tune): cut the motor but let the controller believe u was sent,
                // so it sees an unexplained speed loss like a real ball.
                double sent = now < virtualShotUntilNs ? 0 : u;
                for (CachedMotor cm : cached) cm.setPower(sent, battery);
                core.commit(u, battery);
                lastU = u;

                double t = (now - startNs) / 1e9;
                boolean ready = false;
                if (m == Mode.VELOCITY && tgt > 0) {
                    double tol = Math.max(p.readyTolFrac * tgt, p.readyTolMin);
                    if (Math.abs(core.getVelocityEstimate() - tgt) < tol) {
                        if (readySinceS < 0) readySinceS = t;
                        ready = (t - readySinceS) * 1000 >= p.readySettleMs;
                    } else {
                        readySinceS = -1;
                    }
                } else {
                    readySinceS = -1;
                }

                State s = new State(t, m, m == Mode.VELOCITY ? tgt : 0, vel, core.getVelocityEstimate(),
                        core.getLoadEstimate(), u, battery, dt * 1000, solveMs, core.getLastIterations(), ready,
                        core.isShotActive(), core.getShotCount());
                state = s;
                if (logging) record(s);

                // fixed-rate sleep; if we overran, don't try to catch up with a burst
                next += periodNs;
                long sleep = next - System.nanoTime();
                if (sleep > 0) {
                    LockSupport.parkNanos(sleep);
                } else {
                    next = System.nanoTime();
                }
            }
        } catch (Throwable t) {
            error = t;
        } finally {
            running = false;
            for (CachedMotor cm : cached) safeZero(cm);
        }
    }

    private void record(State s) {
        float[] row = {(float) s.time, (float) s.target, (float) s.velocity, (float) s.estimate,
                (float) s.load, (float) s.power, (float) s.battery, (float) s.loopMs, (float) s.solveMs};
        synchronized (logLock) {
            if (log.length == 0) return;
            log[logHead] = row;
            logHead = (logHead + 1) % log.length;
            if (logCount < log.length) logCount++;
        }
    }

    private double readVelocity() {
        if (hub != null) {
            LynxModule.BulkData b = hub.getBulkData();
            return encoderSign * b.getMotorVelocity(encoderPort);
        }
        return encoderMotor.getVelocity();
    }

    private double readBattery(double fallback) {
        try {
            if (hub != null) {
                double v = hub.getInputVoltage(VoltageUnit.VOLTS);
                if (v > 5) return v;
            }
            for (com.qualcomm.robotcore.hardware.VoltageSensor vs : owner.hardwareMap.voltageSensor) {
                double v = vs.getVoltage();
                if (v > 5) return v;
            }
        } catch (Exception ignored) {
        }
        return fallback;
    }

    private boolean ownerSeenActive = false;

    /**
     * False once the OpMode that created us is no longer the active one. Only trusts the check after
     * it has seen the owner active once, so an SDK quirk can never stop the launcher at init.
     */
    private boolean ownerStillActive() {
        try {
            OpModeManagerImpl mgr = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
            if (mgr == null) return true; // cannot tell; rely on close()
            boolean active = mgr.getActiveOpMode() == owner;
            if (active) ownerSeenActive = true;
            return active || !ownerSeenActive;
        } catch (Exception e) {
            return true;
        }
    }

    private static void safeZero(CachedMotor m) {
        try {
            m.raw().setPower(0);
        } catch (Exception ignored) {
        }
    }

    /** The hub the motor is on (LynxController.getModule() is protected, so via reflection). */
    private static LynxModule findHub(DcMotorEx motor) {
        try {
            Object ctrl = motor.getController();
            if (!(ctrl instanceof LynxController)) return null;
            Method gm = LynxController.class.getDeclaredMethod("getModule");
            gm.setAccessible(true);
            Object mod = gm.invoke(ctrl);
            return mod instanceof LynxModule ? (LynxModule) mod : null;
        } catch (Exception e) {
            return null;
        }
    }
}
