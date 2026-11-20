package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Field-centric mecanum TeleOp.
 *
 * Gamepad 1:
 *   Left stick      translate (field-centric: up = away from the driver)
 *   Right stick X   rotate
 *   Right trigger   slow mode (analog: the further you press, the slower)
 *   Left bumper     hold for robot-centric driving
 *   Y               toggle closed-loop (hub velocity PIDF) vs open-loop drive — needs encoders
 *   Options         reset heading — point the robot away from the driver first
 * Gamepad 2:
 *   Left trigger    rotate turret left
 *   Right trigger   rotate turret right
 */
@TeleOp(name = "Scrimmage", group = "Drive")
public class ScrimmageTeleop extends LinearOpMode {

    static final double DEADBAND = 0.05;
    static final double SLOW_MODE_MIN_POWER = 0.35;
    static final double TURN_SCALE = 0.8;
    private static final double TURRET_GEAR_RATIO = 10.0 / 3.0;
    private DcMotor turret;
    private DcMotor launcher;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Robot front should face away from the driver.");
        telemetry.update();

        waitForStart();

        long lastLoop = System.nanoTime();
        double loopMsAvg = 0.0;

        while (opModeIsActive()) {
            // One bulk read + one IMU read per loop; everything below uses the cached values.
            drive.update();

            long now = System.nanoTime();
            double loopMs = (now - lastLoop) / 1e6;
            lastLoop = now;
            loopMsAvg = loopMsAvg == 0.0 ? loopMs : loopMsAvg * 0.9 + loopMs * 0.1;

            if (gamepad1.optionsWasPressed()) {
                drive.resetHeading();
                gamepad1.rumble(200);
            }

            // Y toggles closed-loop (hub velocity PIDF) vs open-loop (raw power). Needs drive
            // encoders; see MecanumDrive.MAX_TICKS_PER_SEC. Toggle while stopped.
            if (gamepad1.yWasPressed()) {
                drive.setClosedLoop(!drive.isClosedLoop());
                gamepad1.rumble(100);
            }

            // Sticks: up is negative Y on the gamepad, so flip it.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = shape(gamepad1.right_stick_x) * TURN_SCALE;

            // Shape the translation as a vector (not per axis) so diagonals stay true.
            double magnitude = Math.hypot(x, y);
            double shaped = shape(Math.min(magnitude, 1.0));
            double scale = magnitude > 1e-6 ? shaped / magnitude : 0.0;
            double strafe = x * scale;
            double forward = y * scale;

            // Slow mode: right trigger blends max power from 1.0 down to SLOW_MODE_MIN_POWER.
            drive.setMaxPower(1.0 - gamepad1.right_trigger * (1.0 - SLOW_MODE_MIN_POWER));

            boolean robotCentric = gamepad1.left_bumper;
            if (robotCentric) {
                drive.driveRobotCentric(forward, strafe, turn);
            } else {
                drive.driveFieldCentric(forward, strafe, turn);
            }

            // A 3:10 motor-to-turret gear pair makes 10/3 motor turns per turret turn.
            double turretPower = gamepad2.right_trigger - gamepad2.left_trigger;
            if (Math.abs(turretPower) < DEADBAND) turretPower = 0.0;
            turret.setPower(turretPower);

            double motorTicksPerRev = turret.getMotorType().getTicksPerRev();
            double turretTicksPerRev = motorTicksPerRev * TURRET_GEAR_RATIO;
            double turretAngleDegrees = turretTicksPerRev > 0.0
                    ? turret.getCurrentPosition() * 360.0 / turretTicksPerRev
                    : 0.0;

            telemetry.addData("Mode", robotCentric ? "ROBOT-centric" : "FIELD-centric");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drive.getHeading()));
            telemetry.addData("Max power", "%.2f", drive.getMaxPower());
            telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMsAvg, 1000.0 / Math.max(loopMsAvg, 0.1));
            telemetry.addData("Battery", "%.2f V", drive.getBatteryVoltage());
            telemetry.addData("Drive loop", drive.isClosedLoop() ? "CLOSED (hub PIDF)" : "OPEN (power)");
            telemetry.addLine("Options = reset heading | LB = robot-centric | RT = slow");
            telemetry.update();
        }

        drive.stop();
        turret.setPower(0.0);
    }

    /** Deadband, then square while keeping sign — finer control near the centre of the stick. */
    static double shape(double v) {
        if (Math.abs(v) < DEADBAND) return 0.0;
        double n = (Math.abs(v) - DEADBAND) / (1.0 - DEADBAND);
        return Math.copySign(n * n, v);
    }
}
