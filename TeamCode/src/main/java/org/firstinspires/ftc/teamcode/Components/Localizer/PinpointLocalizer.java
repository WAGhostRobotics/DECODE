package org.firstinspires.ftc.teamcode.Components.Localizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer {
    GoBildaPinpointDriver pinpoint;
    private final double xOffset = -122;
    private final double yOffset = 0;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(xOffset, yOffset);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

    }

    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }
    public double getPosX() {
        return pinpoint.getPosX();
    }

    public double getPosY() {
        return pinpoint.getPosY();
    }

    public double getHeading() {
        return pinpoint.getHeading();
    }

    public void update() {
        pinpoint.update();
    }

    public double getEncoderX() {
        return pinpoint.getEncoderX();
    }

    public double getEncoderY() {
        return pinpoint.getEncoderY();
    }

    public void setPose(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    public void setHeadingDegrees(double heading) {
        Pose2D pose = new Pose2D(DistanceUnit.INCH, getPosX(), getPosY(), AngleUnit.DEGREES, heading);
        pinpoint.setPosition(pose);
    }
}
