package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;

/**
 * Registers the Pedro tuners with the tuning web UI. Run them in this order:
 *   1. Mecanum Tuner          → motor directions   → drivetrainConfig
 *   2. Three Wheel IMU Tuner  → pods + IMU         → localizerConfig
 *   3. Tests → Localization   → drive around, check the pose reads correctly
 *   4. Foresight Tuner        → path following     → foresightConfig
 *   5. Tests → Line / Curve / Hold
 * Paste each tuner's printed code block into {@link Constants}.
 */
public class Tuning {

    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Three Wheel IMU Tuner")
    public static Procedure threeWheelIMU() {
        return new ThreeWheelIMUTuner();
    }

    @Tuner(name = "3. Tests")
    public static Procedure tests() {
        // Path/hold tests need a tuned Foresight; pass null until then so Tests rejects them cleanly.
        return new Tests(Constants::drivetrain, Constants::localizer,
                Constants.foresightConfig == null ? null : Constants::algorithm);
    }

    @Tuner(name = "4. Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }
}
