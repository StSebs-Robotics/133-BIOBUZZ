package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Pedro Pathing 3 robot description. One place for the drivetrain, the localizer and the
 * path-following algorithm; everything else calls {@link #createFollower(HardwareMap)}.
 *
 * Motor names, directions and IMU mounting are taken from {@link MecanumDrive} so the two drive
 * stacks can never disagree. Values marked TUNE are placeholders: replace them with the code
 * block the matching tuner prints (Driver Station → "Pedro Tuning" in the tuning web UI).
 */
public class Constants {

    // ---------------------------------------------------------------------------------------
    // Drivetrain — MecanumTuner confirms these directions.
    // ---------------------------------------------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(MecanumDrive.FRONT_LEFT);
        c.frontRightName.set(MecanumDrive.FRONT_RIGHT);
        c.backLeftName.set(MecanumDrive.BACK_LEFT);
        c.backRightName.set(MecanumDrive.BACK_RIGHT);
        c.frontLeftDirection.set(MecanumDrive.FRONT_LEFT_DIR);
        c.frontRightDirection.set(MecanumDrive.FRONT_RIGHT_DIR);
        c.backLeftDirection.set(MecanumDrive.BACK_LEFT_DIR);
        c.backRightDirection.set(MecanumDrive.BACK_RIGHT_DIR);
    });

    // ---------------------------------------------------------------------------------------
    // Localizer — three dead wheels + the hub IMU for heading ("Three Wheel IMU Tuner").
    //
    // Each pod plugs into a motor's encoder port and is named after that motor. The names
    // below are guesses — change them to whichever drive-motor ports the pods actually use.
    // Pedro coordinates: +X forward, +Y left, inches. leftPodY / rightPodY = how far each
    // forward pod is left (+) / right (-) of the centre of rotation; strafePodX = how far the
    // strafe pod is forward (+) / back (-) of it.
    // ---------------------------------------------------------------------------------------
    public static ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {
        c.leftEncoderName.set(MecanumDrive.FRONT_LEFT);    // TUNE: port of the left pod
        c.rightEncoderName.set(MecanumDrive.BACK_RIGHT);   // TUNE: port of the right pod
        c.strafeEncoderName.set(MecanumDrive.BACK_LEFT);   // TUNE: port of the strafe pod
        c.imuName.set(MecanumDrive.IMU_NAME);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(MecanumDrive.HUB_LOGO, MecanumDrive.HUB_USB)));

        c.leftPodY.set(6.0);       // TUNE
        c.rightPodY.set(-6.0);     // TUNE
        c.strafePodX.set(-2.0);    // TUNE

        // goBILDA 4-bar pod (2000 CPR, 32 mm wheel): PI * 32 / 25.4 / 2000 ≈ 0.001979 in/tick.
        c.forwardTicksToInches.set(0.001979);   // TUNE
        c.strafeTicksToInches.set(0.001979);    // TUNE
        c.turnTicksToRadians.set(0.001979);     // TUNE

        c.leftEncoderDirection.set(Encoder.FORWARD);    // TUNE
        c.rightEncoderDirection.set(Encoder.FORWARD);   // TUNE
        c.strafeEncoderDirection.set(Encoder.FORWARD);  // TUNE
    });

    // ---------------------------------------------------------------------------------------
    // Path following — paste the ForesightTuner output here. Until then this stays null:
    // manual (TeleOp) driving and localization work, but follow()/hold() are unavailable.
    // ---------------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = null;

    // ---------------------------------------------------------------------------------------

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new ThreeWheelIMULocalizer(hardwareMap, localizerConfig);
    }

    /** Null until {@link #foresightConfig} has been tuned. */
    public static Algorithm algorithm() {
        return foresightConfig == null ? null : new Foresight(foresightConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        // Encoders and motor writes go through the hubs; one bulk read per loop keeps it fast.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}
