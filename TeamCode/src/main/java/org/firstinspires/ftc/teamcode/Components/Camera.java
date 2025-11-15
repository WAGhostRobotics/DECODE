package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.G;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.P;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Core.Bob;

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
    private double localizerX, localizerY, localizerHeading, lastLocalizerX, lastLocalizerY, estimatedX, estimatedY;
    private double distance, distanceInches;
    private double calculatedShooterVelocity;

    private double targetHeading, headingError, turretAngle, hoodAngle;
    private final double permissibleError = 0.5;



    private final double theta = 55;    // Shooting angle is fixed at 55 degrees
    private final double goalHeight = 1.7;
    private final double limelightAngle = 18;       // Limelight is mounted at 18 degree angle

    // Experimental slope for the line of best fit (relationship between distance and required flywheel velocity)
    private final double experimentalSlope = 0.882431;

    // xTranslation and yTranslation are offsets
    // When limelight sees april tag and estimates pose, the origin is the center of the field
    // We want the origin (0, 0) to be the actual goal
    // So we add these x and y translational offsets to whatever the limelight returns
    // x Translation is the same for red and blue
    // y Translation is positive for blue negative for red
    private double xTranslation = 1.1;
    private double yTranslation = 1.57;

    // Translational constant from the april Tag to the actual backboard

    ElapsedTime timer = new ElapsedTime();
    private final int timerThreshold = 3;           // In seconds
    private int motifID = 0;
    private Intake.SlotState[] motif;
    boolean blueAlliance = true;

    public Camera(HardwareMap hardwareMap, boolean blueAlliance) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        timer.reset();
        limelight3A.start();
        this.blueAlliance = blueAlliance;
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
            aprilX = (botPose.getPosition().x + xTranslation);
            aprilY = (botPose.getPosition().y + yTranslation);

            aprilXInches = aprilX * meterToInches;
            aprilYInches = aprilY * meterToInches;

            distance = Math.hypot(aprilX, aprilY);
            distanceInches = distance * meterToInches;

            // Always relocalize when April Tag is in sight (Timer added to chill the loop speeds and pinpoint death)
            if (timer.seconds()>timerThreshold) {
                Bob.localizer.setPositionOnly(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, netAngle));
                timer.reset();
            }


            // Heading Control to keep Robot locked to the goal
            targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(aprilYInches, aprilXInches)))-180;

        }
        else {
            aprilVisible = false;

            estimatedX = localizerX;
            estimatedY = localizerY;
            distance = Math.hypot(estimatedX/meterToInches, estimatedY/meterToInches);

            distanceInches = distance * meterToInches;
            targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(estimatedY, estimatedX))-180);
        }
        hoodAngle = (Math.max(Math.min(Math.toDegrees(Math.atan(goalHeight/distance)), 63), 27));
        turretAngle = targetHeading - heading;
        turretAngle = normalizeTurretAngle(turretAngle);
        calculatedShooterVelocity = calculateShooterTargetVelocity(distance);
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
                        "TurretAngle: " + turretAngle + "\n";

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

        if (rawX < 1.4) {
            return 128;
        }
        double x = distanceFunction(rawX, theta);
        double rawY = experimentalSlope*x;
        calculatedShooterVelocity = Math.sqrt(rawY)*100;
        return calculatedShooterVelocity;     // Velocity
    }

    private void getLocalizerValues() {
        localizerHeading = Bob.localizer.getHeading();
        localizerY = Bob.localizer.getPosY();
        localizerX = Bob.localizer.getPosX();
    }

    public double getShooterVelocity() {
        return calculatedShooterVelocity;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getAprilHeading() {
        return aprilHeading;
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
            Bob.localizer.setPose(new Pose2D(DistanceUnit.INCH, localizerX, localizerY, DEGREES, normalizeDegrees(aprilHeading-90)));

        }
    }

    public double normalizeTurretAngle(double degrees) {
            return ((degrees + 90) % 360 ) - 90;
    }


    public void switchToMotifPipeline() {
        limelight3A.pipelineSwitch(2);
    }

    public void switchToGoalPipeline() {
        if (!blueAlliance) {
            limelight3A.pipelineSwitch(1);              // Blue april tag Pipeline
            xTranslation *= -1;                               // Flipped bc red is other side
        }
        else {
            limelight3A.pipelineSwitch(0);              // Red april tag Pipeline
        }
    }
    public Intake.SlotState[] getMotif() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {       // If April tag is visible
            motifID = llResult.getFiducialResults().get(0).getFiducialId();
            if (motifID == 21) {
                motif = new Intake.SlotState[]{G, P, P};
            }
            else if (motifID == 22) {
                motif = new Intake.SlotState[]{P, G, P};
            }
            else if (motifID == 23) {
                motif = new Intake.SlotState[]{P, P, G};
            }
        }
        return motif;

    }
}
