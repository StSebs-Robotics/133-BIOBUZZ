package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Four-wheel mecanum drivetrain with an IMU for field-centric control.
 *
 * Coordinate convention (used everywhere in this class):
 *   forward  (+) = away from the driver / robot front
 *   strafe   (+) = to the right
 *   turn     (+) = clockwise (turn right)
 *   heading  (+) = counter-clockwise, radians, 0 = the direction the robot faced at the last reset
 *
 * Usage:
 *   MecanumDrive drive = new MecanumDrive(hardwareMap);
 *   drive.driveFieldCentric(forward, strafe, turn);   // call every loop
 */
public class MecanumDrive {

    // ---------------------------------------------------------------------------------------
    // Configuration — change these to match the robot. Names must match the Driver Station
    // robot configuration exactly.
    // ---------------------------------------------------------------------------------------
    // Confirmed on the robot: back left = motor0. The others are from the Driver Station config;
    // check them (and the directions below) with the "Drive Motor Test" OpMode (tuning group).
    public static final String FRONT_LEFT  = "motor1";   // LF
    public static final String FRONT_RIGHT = "motor5";   // RF
    public static final String BACK_LEFT   = "motor0";   // LB
    public static final String BACK_RIGHT  = "motor4";   // RB
    public static final String IMU_NAME    = "imu";

    /**
     * Directions chosen so that positive power spins every wheel FORWARD (the top of the wheel
     * moves toward the robot's front). Usually the left side is reversed. Used by this class, the
     * Pedro constants and the Drive Motor Test, so a fix here applies everywhere.
     */
    public static final DcMotorSimple.Direction FRONT_LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIR   = DcMotorSimple.Direction.FORWARD;   // checked on the robot: back motors are mounted opposite to the fronts
    public static final DcMotorSimple.Direction BACK_RIGHT_DIR  = DcMotorSimple.Direction.REVERSE;   // checked on the robot

    /** How the Control/Expansion Hub is mounted on the robot. */
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /**
     * Mecanum wheels strafe less efficiently than they drive forward. Bump this above 1.0
     * (typically 1.0–1.2) if the robot strafes noticeably slower than it drives.
     */
    public static final double STRAFE_MULTIPLIER = 1.1;

    /**
     * Top wheel speed in encoder ticks/sec, used only in closed-loop drive mode to turn a -1..1
     * command into a velocity target. MEASURE THIS: run a wheel at full power in RUN_USING_ENCODER
     * and read getVelocity(). Rough starting points: goBILDA 435 rpm + 537.7 ppr ≈ 3900;
     * 312 rpm ≈ 2800; 223 rpm ≈ 2000. Too high and full stick saturates (loses the closed loop);
     * too low and the robot never reaches its real top speed.
     */
    public static final double MAX_TICKS_PER_SEC = 2800.0;

    // ---------------------------------------------------------------------------------------

    /** Battery voltage is a separate hub transaction (not in bulk data), so poll it slowly. */
    public static final long VOLTAGE_POLL_MS = 500;

    /**
     * Custom motor power parameters, shared by all four drive motors. Edit these (or mutate
     * {@link #motorParams} at runtime) to tune how power is applied. See {@link CachedMotor}.
     */
    public final CachedMotor.Params motorParams = new CachedMotor.Params();

    private final List<LynxModule> hubs;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final CachedMotor fl, fr, bl, br;
    private final IMU imu;
    private final VoltageSensor battery;

    /** Per-loop caches, refreshed in {@link #update()}. */
    private double cachedYaw = 0.0;
    private double batteryVoltage = 12.0;
    private long lastVoltageNanos = 0;

    /** Overall power cap applied after normalisation (0..1). */
    private double maxPower = 1.0;

    /**
     * When true, drive commands are sent as velocity targets (ticks/sec) and the hub's on-board
     * PIDF loop closes the loop; when false, raw power. Off by default so behaviour is unchanged
     * until {@link #setClosedLoop(boolean)} is called.
     */
    private boolean closedLoop = false;

    /** Added to the IMU yaw so the field "forward" can be re-zeroed without resetting the IMU. */
    private double headingOffset = 0.0;

    public MecanumDrive(HardwareMap hardwareMap) {
        // Bulk reads in MANUAL mode: exactly one bulk transaction per hub per loop, cleared in
        // update(). AUTO can silently re-fetch if the same value is read twice in one loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        motorParams.writeThreshold = 0.005;   // skip hub writes for changes < 0.5%
        motorParams.slewRatePerSec = 0.0;     // e.g. 6.0 to soften jerky starts; 0 = off
        motorParams.kStatic = 0.0;            // e.g. 0.05 if the robot won't creep at low stick
        motorParams.nominalVoltage = 12.0;
        motorParams.voltageCompensation = false;

        frontLeft  = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLeft.setDirection(FRONT_LEFT_DIR);
        backLeft.setDirection(BACK_LEFT_DIR);
        frontRight.setDirection(FRONT_RIGHT_DIR);
        backRight.setDirection(BACK_RIGHT_DIR);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        fl = new CachedMotor(frontLeft, motorParams);
        fr = new CachedMotor(frontRight, motorParams);
        bl = new CachedMotor(backLeft, motorParams);
        br = new CachedMotor(backRight, motorParams);

        battery = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = battery.getVoltage();
        lastVoltageNanos = System.nanoTime();

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();
        update();
    }

    /**
     * Call exactly once at the top of every loop. Clears the bulk cache and does the loop's only
     * IMU read (an I2C transaction — the most expensive call in the loop). Battery voltage is
     * refreshed every {@link #VOLTAGE_POLL_MS}.
     */
    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        cachedYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        long now = System.nanoTime();
        if (now - lastVoltageNanos > VOLTAGE_POLL_MS * 1_000_000L) {
            batteryVoltage = battery.getVoltage();
            lastVoltageNanos = now;
        }
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    // ---------------------------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------------------------

    /**
     * Drive relative to the field: pushing "forward" always moves away from the driver,
     * regardless of which way the robot is facing.
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Rotate the field-frame (strafe, forward) vector by -heading into the robot frame.
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe  = forward * sin + strafe * cos;

        driveRobotCentric(robotForward, robotStrafe, turn);
    }

    /** Drive relative to the robot: "forward" is wherever the robot's front is pointing. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        strafe *= STRAFE_MULTIPLIER;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale all four down together if any exceeds 1, so the direction of travel is kept.
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        double scale = maxPower / max;
        if (closedLoop) {
            // -1..1 command -> velocity target in ticks/sec; the hub's PIDF loop does the control.
            this.fl.setVelocity(fl * scale * MAX_TICKS_PER_SEC);
            this.fr.setVelocity(fr * scale * MAX_TICKS_PER_SEC);
            this.bl.setVelocity(bl * scale * MAX_TICKS_PER_SEC);
            this.br.setVelocity(br * scale * MAX_TICKS_PER_SEC);
        } else {
            this.fl.setPower(fl * scale, batteryVoltage);
            this.fr.setPower(fr * scale, batteryVoltage);
            this.bl.setPower(bl * scale, batteryVoltage);
            this.br.setPower(br * scale, batteryVoltage);
        }
    }

    public void stop() {
        if (closedLoop) {
            fl.setVelocity(0);
            fr.setVelocity(0);
            bl.setVelocity(0);
            br.setVelocity(0);
            return;
        }
        double slew = motorParams.slewRatePerSec;
        motorParams.slewRatePerSec = 0.0;   // stop immediately, never ramp down
        driveRobotCentric(0, 0, 0);
        motorParams.slewRatePerSec = slew;
    }

    // ---------------------------------------------------------------------------------------
    // Closed-loop (hub velocity PIDF) mode
    // ---------------------------------------------------------------------------------------

    /**
     * Switch all four drive motors between raw power (open loop) and velocity control (closed loop).
     *
     * In closed loop the motors run in RUN_USING_ENCODER and the hub's own velocity PIDF loop holds
     * the commanded speed — the REV firmware confirms this loop runs on the hub's M4 (p + i + d +
     * feedforward) independently of the comm link, so it costs no extra packets per loop. The payoff
     * is consistent speed as the battery drains and far better straight-line tracking; the cost is
     * you must have working drive encoders, set {@link #MAX_TICKS_PER_SEC}, and tune the PIDF.
     *
     * REQUIRES encoders wired to all four drive motors. Default is open loop.
     */
    public void setClosedLoop(boolean enabled) {
        this.closedLoop = enabled;
        DcMotor.RunMode mode = enabled
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(mode);
        }
        if (enabled) {
            applyDefaultVelocityPIDF();
        }
    }

    public boolean isClosedLoop() {
        return closedLoop;
    }

    /**
     * Feedforward-led default for the hub velocity loop: F carries the bulk of the output and P
     * trims the error. F ≈ 32767 / MAX_TICKS_PER_SEC (32767 = full hub drive), P ≈ 0.1·F. Call
     * {@link #setVelocityPIDF} afterwards if you want to hand-tune.
     */
    public void applyDefaultVelocityPIDF() {
        double f = 32767.0 / MAX_TICKS_PER_SEC;
        setVelocityPIDF(0.1 * f, 0.0, 0.0, f);
    }

    /** Zero the drive encoders, then restore the current run mode. Call while stopped. */
    public void resetEncoders() {
        DcMotor.RunMode mode = closedLoop
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(mode);
        }
    }

    /**
     * Velocity-loop PIDF that runs on the hub's M4 (only effective in RUN_USING_ENCODER / closed
     * loop). Sent once; it costs nothing per loop. See {@link #applyDefaultVelocityPIDF} for sane
     * feedforward-led defaults.
     */
    public void setVelocityPIDF(double p, double i, double d, double f) {
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    /** Caps output power, e.g. 0.35 for a precision/slow mode. Clamped to 0..1. */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.0, Math.min(1.0, maxPower));
    }

    public double getMaxPower() {
        return maxPower;
    }

    // ---------------------------------------------------------------------------------------
    // Heading
    // ---------------------------------------------------------------------------------------

    /** Robot heading in radians, CCW positive, normalised to (-PI, PI]. Uses the yaw cached by update(). */
    public double getHeading() {
        return AngleUnit.normalizeRadians(cachedYaw + headingOffset);
    }

    /** Make the robot's current facing direction the new field "forward". */
    public void resetHeading() {
        setHeading(0.0);
    }

    /**
     * Tell the drive what the robot's current heading is in field terms. Useful when the robot
     * starts a match facing sideways or backwards relative to the driver, e.g.
     * {@code setHeading(Math.toRadians(90))} if it starts facing the driver's left.
     */
    public void setHeading(double headingRadians) {
        headingOffset = headingRadians - cachedYaw;
    }
}
