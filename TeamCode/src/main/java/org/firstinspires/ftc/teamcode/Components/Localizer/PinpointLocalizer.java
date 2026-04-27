package org.firstinspires.ftc.teamcode.Components.Localizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class PinpointLocalizer {
    GoBildaPinpointDriver pinpoint;
    long lastTime = System.nanoTime();
    public final static double xOffset = -3.69;
    public final static double yOffset = -1.879;
    // -4, -3.8
    private double lastX = 0, lastY = 0, xVelocity, yVelocity;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }
    public double getPosX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getPosY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void update() {
        long now = System.nanoTime();
        long deltaT = now-lastTime;
        if (deltaT > 33_000_000) {
            pinpoint.update();
            double x = getPosX();
            double y = getPosY();
            xVelocity = (x - lastX)/(deltaT) * 1e9;
            yVelocity = (y - lastY)/(deltaT) * 1e9;
            lastX = x;
            lastY = y;
            lastTime = now;
        }
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

    public void setPositionOnly(Pose2D pose) {
        pinpoint.setPosX(pose.getX(DistanceUnit.INCH), DistanceUnit.INCH);
        pinpoint.setPosY(pose.getY(DistanceUnit.INCH), DistanceUnit.INCH);
    }

    public void setHeadingDegrees(double heading) {
        Pose2D pose = new Pose2D(DistanceUnit.INCH, getPosX(), getPosY(), AngleUnit.DEGREES, heading);
        pinpoint.setPosition(pose);
    }

    public double getXVelocity() {
        return xVelocity;
    }
    public double getYVelocity() {
        return yVelocity;
    }

    public void setOffsets(double x, double y) {
        pinpoint.setOffsets(x, y, DistanceUnit.INCH);
    }

    public double getAngularVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

}
