package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
}
