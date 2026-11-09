package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /**
     * Four goBILDA bare motors, each 20T -> 64T GT2 then 16T -> 56T HTD5 (11.2:1, ~536 rpm) to a 104 mm mecanum
     * wheel. The directions are a guess: replace this with the MecanumTuner output.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /**
     * goBILDA 4-bar pods. Offsets from the CAD (2026-10-27), relative to the center of the four wheels: the forward
     * pod is 3.16 in. left of center, the strafe pod 3.13 in. behind it. Directions are a guess: replace this with
     * the PinpointTuner output.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.16);
        c.yPodOffset.set(-3.13);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static Follower create(HardwareMap h) {
        // Once ForesightTuner has run:
        // return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig),
        //         <Foresight algorithm from ForesightTuner>);
        return null;
    }
}
