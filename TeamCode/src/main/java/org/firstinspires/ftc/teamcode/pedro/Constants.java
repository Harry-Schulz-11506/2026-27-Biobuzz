package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
         return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig));
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.112196554349163);
        c.yPodOffset.set(1.5627025994728871);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3339677194054039);
                Controller secondaryTranslationalForward = Controller.proportional(0.1233921485384826);
                Controller primaryTranslationalLateral = Controller.proportional(0.5699610353188144);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21058537291094995);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012358746096083487));
                c.brake.set(Controller.proportionalFeedforward(0.010504934181670963));

                c.headingFeedback.set(Controller.proportional(7.643430400998451));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04622143645347716, 0.0055412410320929695));

                c.linearBrakeCoefficients.set(Matrix.diag(0.117628761536249, 0.08468067276184202));
                c.quadraticBrakeCoefficients.set(Matrix.diag(6.623791010932842E-4, 0.001293638682017036));

                c.maxAchievableForwardVelocity.set(76.391849335647);
                c.maxAchievableStrafeVelocity.set(61.805053754776466);
                c.naturalForwardDeceleration.set(33.781611555772116);
                c.naturalStrafeDeceleration.set(63.50296628969805);
            }
    );


}