package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherMPC;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;

import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.SLOW_MODE_MIN_POWER;
import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.TURN_SCALE;
import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.shape;

/**
 * Field-centric drive + launcher.
 *
 * Gamepad 1 (driver), same as Field Centric Drive:
 *   Left stick      translate (field-centric: up = away from the driver)
 *   Right stick X   rotate
 *   Right trigger   slow mode (analog)
 *   Left bumper     hold for robot-centric driving
 *   Options         reset heading (point the robot away from the driver first)
 *
 * Gamepad 2 (operator), launcher:
 *   A               launcher on / off (spins to the current speed; off = coast)
 *   X / Y           NEAR / FAR preset speed
 *   Dpad up/down    speed +/- 50 ticks/s       Dpad right/left   +/- 10 ticks/s
 *   Rumbles once each time the launcher reaches speed (ready to shoot).
 *
 * The speed can be changed while it's spinning; it re-targets immediately. Presets and step sizes
 * are the constants below.
 */
@TeleOp(name = "Drive + Launcher", group = "Drive")
public class DriveLauncherTeleOp extends LinearOpMode {

    // ---------------------------------------------------------------- launcher speeds (ticks/s)
    /** Speed selected at start. */
    static final double START_SPEED = 1500;
    static final double NEAR_SPEED = 1400;
    static final double FAR_SPEED = 1700;
    static final double COARSE_STEP = 50;
    static final double FINE_STEP = 10;
    /** Highest speed the operator can select, as a fraction of the launcher's top speed. */
    static final double MAX_FRACTION_OF_TOP = 0.95;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        LauncherMPC launcher = new LauncherMPC(this);

        LauncherParams lp = launcher.getParams();
        double topSpeed = lp.kV - lp.kS;              // at 12 V, from SysId (or defaults)
        double maxSpeed = MAX_FRACTION_OF_TOP * topSpeed;
        double recommended = 0.85 * topSpeed;         // above this, slow shot recovery
        double speed = clamp(START_SPEED, 0, maxSpeed);
        boolean launcherOn = false;
        boolean wasReady = false;

        telemetry.addLine("Ready. Robot front should face away from the driver.");
        if (!launcher.getParamsFile().exists()) {
            telemetry.addLine("⚠ Launcher not tuned yet: run Launcher SysId + Auto-Tune.");
        }
        telemetry.update();
        waitForStart();

        long lastLoop = System.nanoTime();
        double loopMsAvg = 0.0;

        while (opModeIsActive()) {
            drive.update();

            long now = System.nanoTime();
            double loopMs = (now - lastLoop) / 1e6;
            lastLoop = now;
            loopMsAvg = loopMsAvg == 0.0 ? loopMs : loopMsAvg * 0.9 + loopMs * 0.1;

            // ------------------------------------------------------------ drive (gamepad 1)
            if (gamepad1.optionsWasPressed()) {
                drive.resetHeading();
                gamepad1.rumble(200);
            }
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = shape(gamepad1.right_stick_x) * TURN_SCALE;
            double magnitude = Math.hypot(x, y);
            double scale = magnitude > 1e-6 ? shape(Math.min(magnitude, 1.0)) / magnitude : 0.0;
            drive.setMaxPower(1.0 - gamepad1.right_trigger * (1.0 - SLOW_MODE_MIN_POWER));
            boolean robotCentric = gamepad1.left_bumper;
            if (robotCentric) {
                drive.driveRobotCentric(y * scale, x * scale, turn);
            } else {
                drive.driveFieldCentric(y * scale, x * scale, turn);
            }

            // ------------------------------------------------------------ launcher (gamepad 2)
            double newSpeed = speed;
            if (gamepad2.xWasPressed()) newSpeed = NEAR_SPEED;
            if (gamepad2.yWasPressed()) newSpeed = FAR_SPEED;
            if (gamepad2.dpadUpWasPressed()) newSpeed += COARSE_STEP;
            if (gamepad2.dpadDownWasPressed()) newSpeed -= COARSE_STEP;
            if (gamepad2.dpadRightWasPressed()) newSpeed += FINE_STEP;
            if (gamepad2.dpadLeftWasPressed()) newSpeed -= FINE_STEP;
            newSpeed = clamp(newSpeed, 0, maxSpeed);
            boolean speedChanged = newSpeed != speed;
            speed = newSpeed;

            if (gamepad2.aWasPressed()) {
                launcherOn = !launcherOn;
                speedChanged = true;
            }
            if (speedChanged) {
                if (launcherOn) {
                    launcher.setTargetVelocity(speed);
                } else {
                    launcher.off();
                }
            }

            boolean ready = launcherOn && launcher.isReady();
            if (ready && !wasReady) gamepad2.rumble(150);   // just reached speed
            wasReady = ready;

            // ------------------------------------------------------------ telemetry
            telemetry.addData("Launcher", "%s   %s", launcherOn ? "ON" : "off",
                    ready ? "READY" : (launcherOn ? "spinning up..." : ""));
            telemetry.addData("Speed", "%.0f / %.0f ticks/s  (%.0f RPM)", launcher.getVelocity(), speed,
                    speed * 60 / lp.ticksPerRev);
            if (speed > recommended) {
                telemetry.addLine(String.format("⚠ above 85%% of top speed (%.0f): slow recovery", recommended));
            }
            telemetry.addData("Shots", launcher.getShotCount());
            if (launcher.getError() != null) telemetry.addLine("LAUNCHER STOPPED: " + launcher.getError());
            telemetry.addData("Drive", robotCentric ? "ROBOT-centric" : "FIELD-centric");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drive.getHeading()));
            telemetry.addData("Battery", "%.2f V", drive.getBatteryVoltage());
            telemetry.addData("Loop", "%.1f ms", loopMsAvg);
            telemetry.addLine("GP2: A on/off  X near  Y far  dpad speed");
            telemetry.update();
        }

        drive.stop();
        launcher.close();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
