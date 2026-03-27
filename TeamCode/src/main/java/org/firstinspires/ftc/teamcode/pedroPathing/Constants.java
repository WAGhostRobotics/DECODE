package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.4)
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.07, 0.02))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.07, 0.0959625346, 0.0015838))
            .centripetalScaling(0)
            .turnHeadingErrorThreshold(0.012);



    public static PathConstraints pathConstraints = new PathConstraints(0.98, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(90.36)
            .yVelocity(67.0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(PinpointLocalizer.xOffset)
            .strafePodX(PinpointLocalizer.yOffset)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        pathConstraints.setTranslationalConstraint(1.2);
        pathConstraints.setHeadingConstraint(Math.toRadians(1.5));
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
