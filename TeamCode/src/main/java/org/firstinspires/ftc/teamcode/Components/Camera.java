package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Core.Gus;

/**
 * This file includes code to automatically shoot at goal
 * This entails the following
 *     - IF APRIL TAG VISIBLE
 *     - Use limelight pose estimation to find distance from the goal
 *     - Determine target velocity of flywheel using an experimentally determined relationship (Desmos link: https://www.desmos.com/calculator/ffcaewmxtr)
 *     - Always relocalize the robot when April Tag is visible (Set the x and y coordinates)
 *     - Use april Tag x and y estimates to find a target Heading
 *     - Heading control PID to lock the robot chassis on the goal
 *     - IF APRIL TAG NOT VISIBLE
 *     - The robot continues to calculate velocity and maintains heading using localizer values
 *
 * IMPORTANT: Localizer needs to be constantly updated outside this class
 *  @author PK
 */

public class Camera {
    private final Limelight3A limelight3A;
    private boolean aprilVisible;

    // All limelight values are initially in meters. Need to convert to inches
    private final double meterToInches = 39.37;
    private double aprilX, aprilY, aprilXInches, aprilYInches, aprilHeading;
    private double lastX, lastY, localizerX, localizerY, localizerHeading, estimatedX, estimatedY, newY, newX;
    private double seconds, airTime;
    private double distance, distanceInches;

    private double targetHeading, headingError, turretAngle = 0, hoodAngle;

    private double xTranslation = 1.7;
    private double yTranslation = 1.3;

    // Translational constant from the april Tag to the actual backboard

    ElapsedTime timer = new ElapsedTime(), speedTimer = new ElapsedTime();
    private final int timerThreshold = 2;           // In seconds
    private int motifID = 0;
    boolean blueAlliance = true;
    boolean initialized;

    public Camera(HardwareMap hardwareMap, boolean blueAlliance) {
        initialized = false;
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        timer.reset();
        speedTimer.reset();
        limelight3A.start();
        this.blueAlliance = blueAlliance;
        switchToGoalPipeline();
//        if (blueAlliance) {
//            limelight3A.pipelineSwitch(0);              // Blue april tag Pipeline
//            yTranslation *= -1;                               // Flipped bc red is other side
//            yGoalTranslation *= -1;
//        }
//        else {
//            limelight3A.pipelineSwitch(1);              // Red april tag Pipeline
//
//        }

    }
    public Camera(HardwareMap hardwareMap) {
        this(hardwareMap, true);               // Just calls the constructor (defaults to blue alliance)
    }
    public void trackAprilTag(double heading, double turretHeading, boolean tracking) {
        getLocalizerValues();
        double netAngle = heading + turretHeading;
        limelight3A.updateRobotOrientation(netAngle);
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {       // If April tag is visible
            aprilVisible = true;                    // Just for telemetry purposes
            Pose3D botPose = llResult.getBotpose_MT2();
            aprilHeading = botPose.getOrientation().getYaw(DEGREES);

            // Get the x and y (Then apply translation to figure out where robot is relative to the goal)
            aprilX = botPose.getPosition().x + xTranslation;
            if (blueAlliance)
                aprilY = (botPose.getPosition().y + yTranslation);
            else
                aprilY = (botPose.getPosition().y - yTranslation);

            aprilXInches = aprilX * meterToInches;
            aprilYInches = aprilY * meterToInches;

            distance = Math.hypot(aprilX, aprilY) * Math.cos(Math.toRadians(19));

            getLeadPose(true);
//
            distance = Math.hypot(newX/meterToInches, newY/meterToInches) * Math.cos(Math.toRadians(19));
            distanceInches = distance * meterToInches;

            // Always relocalize when April Tag is in sight (Timer added to chill the loop speeds and pinpoint death)
            if (!initialized  || timer.seconds()>timerThreshold) {
                initialized = true;
                Gus.localizer.setPositionOnly(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, netAngle));
                timer.reset();
            }


            // Heading Control to keep Robot locked to the goal
            targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(newY, newX))-180);

        }
        else if (initialized) {

            aprilVisible = false;

            estimatedX = localizerX;
            estimatedY = localizerY;
            getLeadPose(false);
//            distance = Math.hypot(estimatedY/meterToInches, estimatedX/meterToInches) * Math.cos(Math.toRadians(19));
            distance = Math.hypot(newX/meterToInches, newY/meterToInches) * Math.cos(Math.toRadians(19));
            distanceInches = distance * meterToInches;

            targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(newY, newX))-180);
        }

        if (initialized) {
            turretAngle = targetHeading - heading;
            turretAngle = normalizeTurretAngle(turretAngle);
        }
    }

    public String getTelemetry() {
        String returnString = "Is Visible? " + aprilVisible + "\n" +
                                "Initialized: " + initialized + "\n";

//        if (aprilVisible) {
//            returnString = returnString + "April X: " + aprilX + "\n" +
//                    "April X (In): " + aprilXInches + "\n" +
//                    "April Y: " + aprilY + "\n" +
//                    "April Y (In): " + aprilYInches + "\n" +
//                    "April Heading: " + aprilHeading + "\n";
//        }


        returnString = returnString + "Localizer X: " + localizerX + "\n" +
                        "Localizer Y: " + localizerY + "\n" +
                        "Lead X: " + newX + "\n" +
                        "Lead Y: " + newY + "\n" +
                        "AirTime: " + airTime + "\n" +
                        "Estimated X: " + estimatedX + "\n" +
                        "Estimated Y: " + estimatedY + "\n" +
                        "Localizer Heading: " + localizerHeading + "\n" +
                        "Distance: " + distance + "\n" +
                        "Target Heading: " + targetHeading + "\n" +
                        "Heading Error: " + headingError + "\n" +
                        "TurretAngle: " + turretAngle + "\n" +
                        "Blue Alliance: " + blueAlliance;


        return returnString;
    }


    // For distance function and Shooter velocity function explanation check Desmos link:
    // https://www.desmos.com/calculator/ffcaewmxtr
    private void getLocalizerValues() {
        localizerHeading = normalizeDegrees(Gus.localizer.getHeading());
        localizerY = Gus.localizer.getPosY();
        localizerX = Gus.localizer.getPosX();
    }

    private void getLeadPose(boolean isVisible) {
        double changeX = localizerX - lastX;
        double changeY = localizerY - lastY;
        airTime = Gus.shooterLUT.getAirTime(distance);
        seconds = speedTimer.seconds();
        distance = Math.hypot(changeX, changeY);

        if (airTime > 0 ) {
            if (isVisible) {
                newX = aprilXInches + (changeX / seconds) * airTime;
                newY = aprilYInches + (changeY / seconds) * airTime;
            }
            else {
                newX = localizerX + (changeX / seconds) * airTime;
                newY = localizerY + (changeY / seconds) * airTime;
            }
        }
        else {
            if (isVisible) {
                newX = aprilXInches;
                newY = aprilYInches;
            }
            else {
                newX = localizerX;
                newY = localizerY;
            }
        }
        lastX = localizerX;
        lastY = localizerY;
        speedTimer.reset();
    }


    public double getTurretAngle() {
        return turretAngle;
    }



    public double getTargetHeading() {
        return targetHeading;
    }


    public void getNewHeading() {
        getLocalizerValues();
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {       // If April tag is visible
            aprilVisible = true;                    // Just for telemetry purposes
            Pose3D botPose = llResult.getBotpose();
            aprilHeading = botPose.getOrientation().getYaw(DEGREES)-180;
            Gus.localizer.setPose(new Pose2D(DistanceUnit.INCH, localizerX, localizerY, DEGREES, normalizeDegrees(aprilHeading-90)));

        }
    }

    public double normalizeTurretAngle(double degrees) {
            return normalizeDegrees(degrees);
    }


    public void switchToMotifPipeline() {
        limelight3A.pipelineSwitch(2);
    }

    public void switchToGoalPipeline() {
        if (!blueAlliance) {
            limelight3A.pipelineSwitch(1);              // Red april tag Pipeline
        }
        else {
            limelight3A.pipelineSwitch(0);              // Blue april tag Pipeline
        }
    }
    public void switchToBothGoalPipeline() {
        limelight3A.pipelineSwitch(2);
    }

    public void switchPipeline() {

    }

    public double getDistance() {
        return distance;
    }

    public boolean isVisible() {
        return aprilVisible;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void setBlueAlliance(boolean blueAlliance) {
        this.blueAlliance = blueAlliance;
    }

    public void setXYTranslation(double x, double y) {
        xTranslation = x;
        yTranslation = y;
    }

    public void resetInitialized() {
        initialized = false;
    }
}
