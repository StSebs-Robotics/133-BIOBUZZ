package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Wraps a DcMotorEx so that setPower() only reaches the hub when the command has actually changed
 * by a meaningful amount, and lets you shape the power with custom parameters.
 *
 * Why: in the SDK (LynxDcMotorController.internalSetMotorPower) every setPower() whose value is not
 * bit-for-bit identical to the last one becomes a LynxSetMotorConstantPowerCommand, and
 * LynxRespondable.send() holds the hub's network lock and blocks until the hub ACKs it. That is one
 * full UART round trip (~1–3 ms) per motor per loop. Joystick noise changes the value every loop, so
 * a 4-motor drive spends most of its loop time waiting on ACKs. Skipping sub-threshold changes
 * removes those round trips without any noticeable change in how the robot drives.
 *
 * Power pipeline, in order:
 *   1. clamp to [-1, 1]
 *   2. slew-rate limit (max change per second) — 0 disables
 *   3. static-friction feedforward kS: non-zero commands get at least |kS| so the motor actually moves
 *   4. voltage compensation: scale by nominalVoltage / batteryVoltage
 *   5. clamp again, then send only if |new − last sent| ≥ writeThreshold (0 and ±1 always send)
 */
public class CachedMotor {

    /** Tunable motor parameters. Mutate freely; changes apply on the next setPower(). */
    public static class Params {
        /** Smallest change in power worth a hub transaction. 0.005 ≈ 0.5% of full scale. */
        public double writeThreshold = 0.005;
        /** Max power change per second (e.g. 4.0 = 0→full in 0.25 s). 0 = unlimited. */
        public double slewRatePerSec = 0.0;
        /** Minimum magnitude applied to any non-zero command to overcome static friction. */
        public double kStatic = 0.0;
        /** Battery voltage the robot was tuned at. Used for voltage compensation. */
        public double nominalVoltage = 12.0;
        /** Scale power by nominalVoltage / battery so behaviour stays consistent as the battery drains. */
        public boolean voltageCompensation = false;
        /**
         * Smallest change in target velocity (encoder ticks/sec) worth a hub transaction, used by
         * {@link CachedMotor#setVelocity}. The hub's own PIDF loop smooths small differences anyway.
         */
        public double velocityWriteThreshold = 5.0;
    }

    private final DcMotorEx motor;
    private final Params params;

    private double lastSent = Double.NaN;     // NaN forces the first write
    private double lastSentVel = Double.NaN;  // last velocity (ticks/sec) sent to the hub
    private double lastRequested = 0.0;       // after slew, before kS / voltage
    private long lastNanos = 0;

    public CachedMotor(DcMotorEx motor, Params params) {
        this.motor = motor;
        this.params = params;
    }

    public DcMotorEx raw() {
        return motor;
    }

    /** @param batteryVoltage latest battery reading; ignored unless voltageCompensation is on. */
    public void setPower(double power, double batteryVoltage) {
        double target = clamp(power);

        long now = System.nanoTime();
        if (params.slewRatePerSec > 0 && lastNanos != 0) {
            double dt = (now - lastNanos) / 1e9;
            double maxStep = params.slewRatePerSec * dt;
            target = lastRequested + clampAbs(target - lastRequested, maxStep);
        }
        lastNanos = now;
        lastRequested = target;

        double out = target;
        if (out != 0.0 && params.kStatic > 0.0) {
            out = Math.copySign(params.kStatic + Math.abs(out) * (1.0 - params.kStatic), out);
        }
        if (params.voltageCompensation && batteryVoltage > 1.0) {
            out *= params.nominalVoltage / batteryVoltage;
        }
        out = clamp(out);

        boolean mustSend = Double.isNaN(lastSent)
                || (out == 0.0 && lastSent != 0.0)
                || (Math.abs(out) == 1.0 && out != lastSent)
                || Math.abs(out - lastSent) >= params.writeThreshold;
        if (mustSend) {
            motor.setPower(out);
            lastSent = out;
        }
    }

    /**
     * Command a target velocity in encoder ticks/sec. The motor must be in RUN_USING_ENCODER so the
     * hub's on-board velocity PIDF loop (confirmed in the REV firmware: p + i + d + feedforward)
     * does the control. Like setPower, this only reaches the hub when the target actually changed by
     * at least {@link Params#velocityWriteThreshold}, so one noisy stick doesn't flood the UART.
     * kS / slew / voltage compensation do not apply here — the closed loop handles those effects.
     */
    public void setVelocity(double ticksPerSec) {
        boolean mustSend = Double.isNaN(lastSentVel)
                || (ticksPerSec == 0.0 && lastSentVel != 0.0)
                || Math.abs(ticksPerSec - lastSentVel) >= params.velocityWriteThreshold;
        if (mustSend) {
            motor.setVelocity(ticksPerSec);
            lastSentVel = ticksPerSec;
        }
    }

    /** Last power value actually sent to the hub. */
    public double getSentPower() {
        return Double.isNaN(lastSent) ? 0.0 : lastSent;
    }

    /** Last velocity target (ticks/sec) actually sent to the hub. */
    public double getSentVelocity() {
        return Double.isNaN(lastSentVel) ? 0.0 : lastSentVel;
    }

    private static double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    private static double clampAbs(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
