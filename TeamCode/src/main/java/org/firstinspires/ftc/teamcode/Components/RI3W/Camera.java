package org.firstinspires.ftc.teamcode.Components.RI3W;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RI3W.George;

/**
 * This file includes code to automatically shoot at goal
 * This entails the following
 *     - IF APRIL TAG VISIBLE
 *     - Use limelight pose estimation to find distance from the goal
 *     - Determine target velocity of flywheel using an experimentally determined relationship (Desmos link: https://www.desmos.com/calculator/ffcaewmxtr)
 *     - Always relocalize the robot when April Tag is visible (Set the x and y coordinates and heading)
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
    private double localizerX, localizerY, localizerHeading, lastLocalizerX, lastLocalizerY, estimatedX, estimatedY;
    private double distance, distanceInches;
    private double calculatedShooterVelocity;

    private double targetHeading, headingError, driveTurn;
    private final double permissibleError = 0.5;
    PIDController headingControl = new PIDController(0.01, 0.000, 0);
    private final double kStaticTurn = 0.05;



    private final double theta = 55;    // Shooting angle is fixed at 55 degrees
    private final double limelightAngle = 18;       // Limelight is mounted at 18 degree angle

    // Experimental slope for the line of best fit (relationship between distance and required flywheel velocity)
    private final double experimentalSlope = 0.948498;

    // xTranslation and yTranslation are offsets
    // When limelight sees april tag and estimates pose, the origin is the center of the field
    // We want the origin (0, 0) to be the actual goal
    // So we add these x and y translational offsets to whatever the limelight returns
    // x Translation is the same for red and blue
    // y Translation is positive for blue negative for red
    private final double xTranslation = 1.482;
    private double yTranslation = 1.413;

    // Translational constant from the april Tag to the actual backboard
    private final double xGoalTranslation = 10;
    private double yGoalTranslation = 13.5;

    public Camera(HardwareMap hardwareMap, boolean blueAlliance) {
        headingControl.setIntegrationBounds(-10000000, 10000000);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.start();
        if (blueAlliance) {
            limelight3A.pipelineSwitch(0);              // Blue april tag Pipeline
        }
        else {
            limelight3A.pipelineSwitch(1);              // Red april tag Pipeline
            yTranslation *= -1;                               // Flipped bc red is other side
            yGoalTranslation *= -1;
        }

    }
    public Camera(HardwareMap hardwareMap) {
        this(hardwareMap, true);               // Just calls the constructor (defaults to blue alliance)
    }

    public void trackAprilTag(double robotHeading) {
        getLocalizerValues();
        limelight3A.updateRobotOrientation(robotHeading);
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {       // If April tag is visible
            aprilVisible = true;                    // Just for telemetry purposes
            Pose3D botPose = llResult.getBotpose();
            aprilHeading = botPose.getOrientation().getYaw(DEGREES);

            // Get the x and y (Then apply translation to figure out where robot is relative to the goal)
            aprilX = (botPose.getPosition().x + xTranslation);
            aprilY = (botPose.getPosition().y + yTranslation);

            aprilXInches = aprilX * meterToInches;
            aprilYInches = aprilY * meterToInches;
            lastLocalizerX = aprilXInches;
            lastLocalizerY = aprilYInches;

            distance = Math.hypot(aprilX, aprilY)*Math.cos(Math.toRadians(limelightAngle));
            distanceInches = distance * meterToInches;

            // Always relocalize when April Tag is in sight
            George.localizer.setPose(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, normalizeDegrees(aprilHeading+90)));


            // Heading Control to keep Robot locked to the goal

            // This is a bit weird. But we want to aim at the goal backboard, not at the april tag.
            // Adding an offset from the april tag to the goal helps the robot aim better
            double aprilXInchesGoal = aprilXInches + xGoalTranslation;
            double aprilYInchesGoal = Math.signum(aprilYInches)*yGoalTranslation + aprilYInches;

            targetHeading = -normalizeDegrees(Math.toDegrees(Math.atan2(aprilXInchesGoal, aprilYInchesGoal)));
            headingError = targetHeading - localizerHeading;
        }
        else {
            aprilVisible = false;
            estimatedX = lastLocalizerX - (lastLocalizerY - localizerY);
            estimatedY = lastLocalizerY + (lastLocalizerX - localizerX);
            distance = Math.hypot(estimatedX/meterToInches, estimatedY/meterToInches);
            estimatedX += (xGoalTranslation);
            estimatedY += (yGoalTranslation);

            distanceInches = distance * meterToInches;
            double angledLocalizerX = estimatedX / Math.cos(Math.toRadians(limelightAngle));
            double angledLocalizerY = estimatedY / Math.cos(Math.toRadians(limelightAngle));



            targetHeading = -normalizeDegrees(Math.toDegrees(Math.atan2(angledLocalizerX, angledLocalizerY)));
            headingError = targetHeading - localizerHeading;
        }

        calculatedShooterVelocity = calculateShooterTargetVelocity(distance);
        driveTurn = headingControl.calculate(0, headingError);
        driveTurn = Math.signum(driveTurn) * kStaticTurn + driveTurn;
        driveTurn = Range.clip(driveTurn, -1, 1);
        if (Math.abs(headingError)<permissibleError) {
            resetHeadingControl();
            driveTurn = 0;
        }
    }

    public String getTelemetry() {
        String returnString = "Is Visible? " + aprilVisible + "\n";

        if (aprilVisible) {
            returnString = returnString + "April X: " + aprilX + "\n" +
                    "April X (In): " + aprilXInches + "\n" +
                    "April Y: " + aprilY + "\n" +
                    "April Y (In): " + aprilYInches + "\n" +
                    "April Heading: " + aprilHeading + "\n";
        }


        returnString = returnString + "Localizer X: " + localizerX + "\n" +
                        "Localizer Y: " + localizerY + "\n" +
                        "Estimated X: " + estimatedX + "\n" +
                        "Estimated Y: " + estimatedY + "\n" +
                        "Localizer Heading: " + localizerHeading + "\n" +
                        "Distance: " + distance + "\n" +
                        "Distance (In) " + distanceInches + "\n" +
                        "Target Shooter Velocity: " + calculatedShooterVelocity + "\n" +
                        "Target Heading: " + targetHeading + "\n" +
                        "Heading Error: " + headingError + "\n" +
                        "DriveTurn: " + driveTurn + "\n";

        return returnString;
    }


    // For distance function and Shooter velocity function explanation check Desmos link:
    // https://www.desmos.com/calculator/ffcaewmxtr
    private double distanceFunction(double x, double theta) {  // Theta in degrees
        double xSquared = Math.pow(x, 2);
        double tanTheta = Math.tan(Math.toRadians(theta));   // Theta converted to radians before than
        return (xSquared)/((x*tanTheta)-1);
    }

    private double calculateShooterTargetVelocity(double rawX) {
        // Caveats:
        // If robot is closer than a certain distance, shoot at an experimentally measured minimum velocity

        if (rawX < 1.5) {
            return 128;
        }
        double x = distanceFunction(rawX, theta);
        double rawY = experimentalSlope*x;
        calculatedShooterVelocity = Math.sqrt(rawY)*100;
        return calculatedShooterVelocity;     // Velocity
    }

    private void getLocalizerValues() {
        localizerHeading = George.localizer.getHeading();
        localizerY = George.localizer.getPosY();
        localizerX = George.localizer.getPosX();
    }

    public double getShooterVelocity() {
        return calculatedShooterVelocity;
    }

    public double getDriveTurn() {
        return driveTurn;
    }

    public double getAprilHeading() {
        return aprilHeading;
    }

    public void resetHeadingControl() {
        headingControl.reset();
    }
}
