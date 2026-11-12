package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/** Run in this order; paste each tuner's output into {@link Constants} before running the next. */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    /**
     * Localization and driving checks. The hold / line / curve tests also need the path-following algorithm:
     * once ForesightTuner has run, replace {@code null} with a supplier of that Foresight algorithm.
     */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                null);
    }
}
