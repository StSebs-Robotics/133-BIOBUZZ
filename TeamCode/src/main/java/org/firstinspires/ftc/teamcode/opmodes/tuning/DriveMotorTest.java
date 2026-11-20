package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Checks the drive wiring one motor at a time. PROP THE ROBOT UP so the wheels are off the ground.
 *
 * Hold a button and only that motor spins, slowly, with the same name and direction the drive
 * code uses. The screen says which wheel SHOULD move and that it should spin FORWARD (the top of
 * the wheel moving toward the robot's front).
 *
 *   Left bumper    front left        Right bumper    front right
 *   Left trigger   back left         Right trigger   back right
 *
 * If the wrong wheel moves: the names are swapped (MecanumDrive.FRONT_LEFT etc.).
 * If the right wheel spins backward: flip that wheel's direction (MecanumDrive.FRONT_LEFT_DIR etc.).
 */
@TeleOp(name = "Drive Motor Test", group = "tuning")
public class DriveMotorTest extends LinearOpMode {

    static final double POWER = 0.3;

    private static final class Wheel {
        final String label, name;
        final DcMotorSimple.Direction dir;
        final DcMotorEx motor;

        Wheel(String label, String name, DcMotorSimple.Direction dir, DcMotorEx motor) {
            this.label = label;
            this.name = name;
            this.dir = dir;
            this.motor = motor;
        }
    }

    @Override
    public void runOpMode() {
        Wheel[] wheels = {
                wheel("FRONT LEFT", MecanumDrive.FRONT_LEFT, MecanumDrive.FRONT_LEFT_DIR),
                wheel("FRONT RIGHT", MecanumDrive.FRONT_RIGHT, MecanumDrive.FRONT_RIGHT_DIR),
                wheel("BACK LEFT", MecanumDrive.BACK_LEFT, MecanumDrive.BACK_LEFT_DIR),
                wheel("BACK RIGHT", MecanumDrive.BACK_RIGHT, MecanumDrive.BACK_RIGHT_DIR),
        };

        telemetry.addLine("DRIVE MOTOR TEST — prop the robot up, wheels off the ground.");
        telemetry.addLine("LB front-left  RB front-right  LT back-left  RT back-right");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean[] held = {
                    gamepad1.left_bumper,
                    gamepad1.right_bumper,
                    gamepad1.left_trigger > 0.5,
                    gamepad1.right_trigger > 0.5,
            };
            int active = -1;
            for (int i = 0; i < wheels.length; i++) {
                if (held[i] && active < 0) active = i; // one motor at a time
            }
            for (int i = 0; i < wheels.length; i++) {
                wheels[i].motor.setPower(i == active ? POWER : 0);
            }

            telemetry.addLine("Hold ONE: LB front-left  RB front-right  LT back-left  RT back-right");
            telemetry.addLine();
            if (active >= 0) {
                Wheel w = wheels[active];
                telemetry.addLine(String.format("Running \"%s\" (%s) at +%.1f power", w.name, w.dir, POWER));
                telemetry.addLine(">> Only the " + w.label + " wheel should spin,");
                telemetry.addLine(">> and FORWARD (top of the wheel toward the robot's front).");
            } else {
                telemetry.addLine("Nothing running.");
            }
            telemetry.addLine();
            for (Wheel w : wheels) {
                telemetry.addLine(String.format("%-12s = \"%s\"  %s", w.label, w.name, w.dir));
            }
            telemetry.addLine();
            telemetry.addLine("Wrong wheel moves → names swapped.  Right wheel, wrong way → flip direction.");
            telemetry.update();
        }
        for (Wheel w : wheels) w.motor.setPower(0);
    }

    private Wheel wheel(String label, String name, DcMotorSimple.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
        return new Wheel(label, name, dir, m);
    }
}
