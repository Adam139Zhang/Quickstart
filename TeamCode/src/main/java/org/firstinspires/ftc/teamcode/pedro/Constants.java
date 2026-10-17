package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig =
            new MecanumConfig(c -> {
                c.frontLeftName.set("leftFront");
                c.frontRightName.set("rightFront");
                c.backLeftName.set("leftBack");
                c.backRightName.set("rightBack");
            });

    public static PinpointConfig localizerConfig =
            new PinpointConfig(c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);

                c.podType.set(
                        GoBildaPinpointDriver.GoBildaOdometryPods
                                .goBILDA_4_BAR_POD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            });

    public static ForesightConfig foresightConfig =
            new ForesightConfig(c -> {
            });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}