package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherMPC;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherParams;

/**
 * Turret speed test: hold the launcher at a set speed and nudge it with the bumpers.
 *
 * Gamepad 1:
 *   A               on / off (off = coast)
 *   Right bumper    speed + STEP
 *   Left bumper     speed - STEP
 *   Rumbles once each time it reaches speed.
 */
@TeleOp(name = "Turret Speed", group = "Test")
public class TurretSpeedTeleOp extends LinearOpMode {

    /** Speed at start, ticks/s. */
    static final double START_SPEED = 1500;
    /** Change per bumper press, ticks/s. */
    static final double STEP = 50;
    /** Highest selectable speed, as a fraction of the launcher's top speed. */
    static final double MAX_FRACTION_OF_TOP = 0.95;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(100);
        LauncherMPC launcher = new LauncherMPC(this);
        LauncherParams lp = launcher.getParams();
        double maxSpeed = MAX_FRACTION_OF_TOP * (lp.kV - lp.kS);

        double speed = Math.min(START_SPEED, maxSpeed);
        boolean on = false;
        boolean wasReady = false;

        telemetry.addLine("Ready. A = on/off, bumpers = speed up/down.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double newSpeed = speed;
            if (gamepad1.rightBumperWasPressed()) newSpeed += STEP;
            if (gamepad1.leftBumperWasPressed()) newSpeed -= STEP;
            newSpeed = Math.max(0, Math.min(maxSpeed, newSpeed));
            boolean changed = newSpeed != speed;
            speed = newSpeed;

            if (gamepad1.aWasPressed()) {
                on = !on;
                changed = true;
            }
            if (changed) {
                if (on) launcher.setTargetVelocity(speed);
                else launcher.off();
            }

            boolean ready = on && launcher.isReady();
            if (ready && !wasReady) gamepad1.rumble(150);
            wasReady = ready;

            telemetry.addData("Turret", "%s   %s", on ? "ON" : "off",
                    ready ? "READY" : (on ? "spinning up..." : ""));
            telemetry.addData("Target", "%.0f ticks/s  (%.0f RPM)", speed, speed * 60 / lp.ticksPerRev);
            telemetry.addData("Actual", "%.0f ticks/s", launcher.getVelocity());
            telemetry.addData("Max", "%.0f ticks/s", maxSpeed);
            if (launcher.getError() != null) telemetry.addLine("STOPPED: " + launcher.getError());
            telemetry.addLine("A on/off   RB +" + (int) STEP + "   LB -" + (int) STEP);
            telemetry.update();
        }

        launcher.close();
    }
}
