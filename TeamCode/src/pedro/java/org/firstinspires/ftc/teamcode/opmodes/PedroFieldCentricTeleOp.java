package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.SLOW_MODE_MIN_POWER;
import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.TURN_SCALE;
import static org.firstinspires.ftc.teamcode.opmodes.FieldCentricTeleOp.shape;

/**
 * Field-centric TeleOp driven through Pedro's Follower, so the odometry pose is tracked the whole
 * time (ready for auto-aim, drive-to-point, or carrying the pose over from autonomous).
 *
 * Same controls as {@link FieldCentricTeleOp}:
 *   Left stick      translate (field-centric: up = away from the driver)
 *   Right stick X   rotate
 *   Right trigger   slow mode
 *   Left bumper     hold for robot-centric driving
 *   Options         reset field "forward" to where the robot is facing (pose is kept)
 */
@TeleOp(name = "Pedro Field Centric Drive", group = "Drive")
public class PedroFieldCentricTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(Pose.zero());

        // Added to the pose heading for field-centric only, so resetting "forward" never
        // corrupts the odometry heading that paths and autonomous rely on.
        double headingOffset = 0.0;

        telemetry.addLine("Ready. Robot front should face away from the driver.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Pose pose = follower.pose();

            if (gamepad1.optionsWasPressed()) {
                headingOffset = -pose.heading();
                gamepad1.rumble(200);
            }

            // Our sticks: up = forward, right = +x, right stick right = turn clockwise.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = shape(gamepad1.right_stick_x) * TURN_SCALE;

            double magnitude = Math.hypot(x, y);
            double scale = magnitude > 1e-6 ? shape(Math.min(magnitude, 1.0)) / magnitude : 0.0;

            double slow = 1.0 - gamepad1.right_trigger * (1.0 - SLOW_MODE_MIN_POWER);

            // Pedro's convention: +strafe = LEFT, +turn = counter-clockwise. Flip ours to match.
            DrivePowers robot = new DrivePowers(y * scale * slow, -x * scale * slow, -turn * slow);

            boolean robotCentric = gamepad1.left_bumper;
            follower.manual(robotCentric
                    ? robot
                    : ManualDrive.fieldCentric(robot, pose.heading(), headingOffset));
            follower.update();

            telemetry.addData("Mode", robotCentric ? "ROBOT-centric" : "FIELD-centric");
            telemetry.addData("X (in)", "%.1f", pose.x());
            telemetry.addData("Y (in)", "%.1f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addLine("Options = reset forward | LB = robot-centric | RT = slow");
            telemetry.update();
        }

        follower.stop();
        follower.update();
    }
}
