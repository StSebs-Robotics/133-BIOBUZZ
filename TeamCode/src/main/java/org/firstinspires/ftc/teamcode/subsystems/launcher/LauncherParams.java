package org.firstinspires.ftc.teamcode.subsystems.launcher;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Properties;

/**
 * Every number the launcher MPC uses. Pure Java (no FTC imports) so the desktop simulator in
 * tools/launcher can load the same file.
 *
 * Values are saved to / loaded from a .properties file on the Control Hub
 * (/sdcard/FIRST/launcher/&lt;name&gt;.properties). The SysId OpMode writes the model section, the
 * MPC Tuner OpMode writes the rest. So for a new launcher you do not edit code: run
 * "Launcher SysId", then "Launcher MPC Tuner", press save, done. The defaults below only matter
 * when no file exists yet.
 *
 * Units: velocity is encoder ticks per second, power is the normal -1..1 motor power.
 */
public class LauncherParams {

    // ---------------------------------------------------------------- plant model (from SysId)
    //   tau * dv/dt = -v + kV * power * (battery / nominalVoltage) - kS - load
    /** Time constant, seconds: time to reach 63% of a step. */
    public double tau = 0.60;
    /** Steady-state ticks/s per unit power at nominalVoltage (slope of the power→speed line). */
    public double kV = 2400;
    /** Friction offset, ticks/s (power→speed line hits 0 at power = kS / kV). */
    public double kS = 80;
    /** Dead time between setPower() and the encoder showing it, seconds (hub latency + velocity filter). */
    public double delaySec = 0.010;
    /** Battery voltage the model was identified at. */
    public double nominalVoltage = 12.0;

    // ---------------------------------------------------------------- loop
    /** Controller thread period, ms. 10 ms = 100 Hz. Each loop is one bulk read + one setPower. */
    public int periodMs = 10;

    // ---------------------------------------------------------------- MPC
    /** How far ahead the MPC looks, seconds. Roughly 0.5–1.5 × tau. */
    public double horizonSec = 0.60;
    /** Number of prediction steps over the horizon (size of the QP). 8–25. */
    public int horizonSteps = 15;
    /** Weight on velocity error. Leave at 1 and tune the others relative to it. */
    public double qVel = 1.0;
    /** Extra weight on the error at the end of the horizon. Higher = less overshoot, firmer settling. */
    public double qTerminal = 5.0;
    /** Weight on power changes. Higher = smoother, slower; lower = snappier, noisier. */
    public double rDu = 0.05;
    /**
     * Power limits. A negative uMin lets the MPC brake (drive backwards) when slowing down; 0 =
     * coast only. Keep it small: reverse power on a fast-spinning flywheel draws a lot of current
     * (the motor's back-EMF adds to the battery). Keep the motor on FLOAT, never BRAKE mode: BRAKE
     * slows it in a way the model doesn't know about.
     */
    public double uMin = -0.15;
    public double uMax = 1.0;
    /** Optional limit on power change per second applied after the MPC (0 = off). */
    public double maxDuPerSec = 0.0;
    /** QP solver iterations per loop (warm-started, so 20–40 is plenty). */
    public int maxIters = 40;

    // ---------------------------------------------------------------- Kalman observer
    /** Std-dev of encoder velocity noise, ticks/s. Higher = trust the encoder less (smoother, slower). */
    public double measNoise = 30;
    /** Std-dev of unmodelled velocity change per loop, ticks/s. */
    public double velNoise = 6;
    /** Std-dev of load change per loop, ticks/s. Higher = recovers from shots / model error faster. */
    public double loadNoise = 4;
    /**
     * Shot detector threshold (CUSUM, in measNoise units). When the speed falls faster than the
     * model allows, it is treated as a ball going through: the speed estimate follows the encoder
     * and the load estimate is frozen, so there is no over-speed after recovery. Lower = more
     * sensitive. 0 = off.
     */
    public double shotDetectSigma = 3;

    // ---------------------------------------------------------------- shot feed-forward (notifyShot)
    /**
     * How much of the expected dip to plan for when your code calls launcher.notifyShot().
     * 1 = all of it, 0 = off. Only used once the timing has been learned.
     */
    public double shotFeedforward = 1.0;
    /** Learned: time from notifyShot() to the ball hitting the wheel, s. 0 = not learned yet. */
    public double shotLeadSec = 0;
    /** Learned: speed lost per ball, fraction of speed. 0 = not learned yet. */
    public double shotDropFrac = 0;

    // ---------------------------------------------------------------- "ready to shoot"
    /** Ready when |error| &lt; max(readyTolFrac * target, readyTolMin) ... */
    public double readyTolFrac = 0.02;
    public double readyTolMin = 20;
    /** ... continuously for this long, ms. */
    public int readySettleMs = 80;

    // ---------------------------------------------------------------- display only
    /** Encoder ticks per flywheel revolution (28 for a bare goBILDA/REV motor encoder × gear ratio). */
    public double ticksPerRev = 28;

    // =============================================================================================

    public LauncherParams copy() {
        LauncherParams c = new LauncherParams();
        try {
            for (Field f : fields()) f.set(c, f.get(this));
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        return c;
    }

    /** Clamp anything that would break the controller. Called by the controller on every update. */
    public LauncherParams sanitized() {
        LauncherParams c = copy();
        c.tau = Math.max(c.tau, 0.01);
        c.kV = Math.max(c.kV, 1);
        c.delaySec = Math.max(0, Math.min(c.delaySec, 0.2));
        c.nominalVoltage = Math.max(c.nominalVoltage, 6);
        c.periodMs = Math.max(2, Math.min(c.periodMs, 50));
        c.horizonSteps = Math.max(2, Math.min(c.horizonSteps, 60));
        c.horizonSec = Math.max(c.horizonSec, c.periodMs / 1000.0 * 2);
        c.qVel = Math.max(c.qVel, 0);
        c.qTerminal = Math.max(c.qTerminal, 0);
        c.rDu = Math.max(c.rDu, 1e-6);
        c.uMax = Math.max(-1, Math.min(c.uMax, 1));
        c.uMin = Math.max(-1, Math.min(c.uMin, c.uMax));
        c.maxIters = Math.max(1, Math.min(c.maxIters, 500));
        c.measNoise = Math.max(c.measNoise, 1e-3);
        c.velNoise = Math.max(c.velNoise, 1e-3);
        c.loadNoise = Math.max(c.loadNoise, 0);
        c.shotDetectSigma = Math.max(c.shotDetectSigma, 0);
        c.shotFeedforward = Math.max(0, Math.min(c.shotFeedforward, 2));
        c.shotLeadSec = Math.max(0, Math.min(c.shotLeadSec, 1));
        c.shotDropFrac = Math.max(0, Math.min(c.shotDropFrac, 0.6));
        return c;
    }

    // ---------------------------------------------------------------- persistence

    public static File fileFor(File dir, String name) {
        return new File(dir, name + ".properties");
    }

    /** Loads the file if it exists; any missing key keeps its default. Never throws. */
    public static LauncherParams loadOrDefault(File file) {
        LauncherParams p = new LauncherParams();
        if (file == null || !file.exists()) return p;
        Properties props = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            props.load(in);
        } catch (IOException e) {
            return p;
        }
        for (Field f : fields()) {
            String s = props.getProperty(f.getName());
            if (s == null) continue;
            try {
                if (f.getType() == double.class) f.setDouble(p, Double.parseDouble(s.trim()));
                else if (f.getType() == int.class) f.setInt(p, (int) Math.round(Double.parseDouble(s.trim())));
                else if (f.getType() == boolean.class) f.setBoolean(p, Boolean.parseBoolean(s.trim()));
            } catch (Exception ignored) {
                // bad value in file: keep default
            }
        }
        return p;
    }

    public void save(File file, String comment) throws IOException {
        File dir = file.getParentFile();
        if (dir != null && !dir.exists() && !dir.mkdirs()) throw new IOException("cannot create " + dir);
        Properties props = new Properties();
        for (Field f : fields()) {
            try {
                props.setProperty(f.getName(), String.valueOf(f.get(this)));
            } catch (IllegalAccessException e) {
                throw new IOException(e);
            }
        }
        try (OutputStream out = new FileOutputStream(file)) {
            props.store(out, comment);
        }
    }

    /** Public, non-static, non-final number/boolean fields, in declaration order. */
    public static Field[] fields() {
        java.util.ArrayList<Field> out = new java.util.ArrayList<>();
        for (Field f : LauncherParams.class.getDeclaredFields()) {
            int m = f.getModifiers();
            if (!Modifier.isPublic(m) || Modifier.isStatic(m) || Modifier.isFinal(m)) continue;
            Class<?> t = f.getType();
            if (t == double.class || t == int.class || t == boolean.class) out.add(f);
        }
        return out.toArray(new Field[0]);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        try {
            for (Field f : fields()) sb.append(f.getName()).append('=').append(f.get(this)).append(' ');
        } catch (IllegalAccessException ignored) {
        }
        return sb.toString().trim();
    }
}
