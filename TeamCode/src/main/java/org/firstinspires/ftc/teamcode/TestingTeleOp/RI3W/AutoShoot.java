package org.firstinspires.ftc.teamcode.TestingTeleOp.RI3W;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RI3W.George;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
@TeleOp
public class AutoShoot extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(AutoShoot.class);
    public static boolean tuning = false;
    private final double theta = 55;
    private final double experimentalSlope = 0.948498;

    public static double shooterVelocity, tuneVelocity;
    public static double intakePower;

    double localizerX, localizerY, localizerHeading;
    public static double xTranslation = 1.482;
    public static double yTranslation = 1.413; // Y is only for Blue. Red would be negative
    public static double P = 0.0008, I = 0.000, D = 0;
    public static double kStaticTurn = 0.05;
    PIDController headingControl;
    private double targetHeading, targetHeadingEstimate, headingError;
    boolean shooterOn = false;
    boolean isFieldOriented = true;
    private double error;

    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader fieldOriented = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader resetHeading = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader reloacalize = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        headingControl = new PIDController(P, I, D);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        double driveTurn = 0;
        double aprilX, aprilY, distance = 0;
        double distanceEstimate = 0;
        double aprilXInches, aprilYInches;
        limelight3A.start();
        limelight3A.pipelineSwitch(0);
        George.init(hardwareMap);

        while (opModeInInit()) {
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();
                double aprilHeading = botPose.getOrientation().getYaw(DEGREES);
                aprilX = (botPose.getPosition().x + xTranslation);
                aprilY = (botPose.getPosition().y + yTranslation);
                aprilXInches = aprilX * 39.37;
                aprilYInches = aprilY * 39.37;
                George.localizer.setPose(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, aprilHeading));
            }
        }

        waitForStart();
        while (opModeIsActive()) {
            George.localizer.update();
            getLocalizerValues();
//            limelight3A.updateRobotOrientation(George.localizer.getHeading());
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double magnitude = Math.hypot(x, y);
            double tx = 0;
            double theta = Math.toDegrees(Math.atan2(y, x));
            if (isFieldOriented) {
                theta = normalizeDegrees(theta - George.localizer.getHeading());
            }
            headingControl.setPID(P, I, D);
            if (shooterOn) {
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    gamepad1.setLedColor(0, 255, 0, 1000);
                    Pose3D botPose = llResult.getBotpose();
                    double aprilHeading = botPose.getOrientation().getYaw(DEGREES);
                    aprilX = (botPose.getPosition().x + xTranslation);
                    aprilY = (botPose.getPosition().y + yTranslation);
                    aprilXInches = aprilX * 39.37;
                    aprilYInches = aprilY * 39.37;
                    targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(aprilYInches, aprilXInches)))-180;
                    headingError = targetHeading - aprilHeading;
                    distance = Math.hypot(aprilX, aprilY)*Math.cos(Math.toRadians(18));
                    shooterVelocity = computeVelocity(distance);
                    if (reloacalize.wasJustReleased()) {
                        George.localizer.setPose(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, aprilHeading));
                    }

//                    tx = -llResult.getTx() + 1.7;     // offset bc limelight not in middle
//                    if (Math.abs(tx) <= 0.75) {
//                        tx = 0;
//                        headingControl.reset();
//                    }
                    telemetry.addData("April Heading: ", aprilHeading);
                    telemetry.addData("Distance: ", distance);
                    telemetry.addData("AprilX Inches: ", aprilXInches);
                    telemetry.addData("AprilY Inches: ", aprilYInches);
                }
                else {
                    distanceEstimate = Math.hypot(localizerX/39.37, localizerY/39.37);
                    double angledLocalizerY = localizerY / Math.cos(Math.toRadians(18));
                    double angledLocalizerX = localizerX / Math.cos(Math.toRadians(18));
                    targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(angledLocalizerY, angledLocalizerX)))-180;
                    headingError = targetHeading - localizerHeading;
                    telemetry.addLine("Localizer Stuff\n\n");
                    shooterVelocity = computeVelocity(distanceEstimate);

                }
                driveTurn = headingControl.calculate(0, headingError);
                driveTurn = Math.signum(driveTurn) * kStaticTurn + driveTurn;
                driveTurn = Range.clip(driveTurn, -1, 1);
                if (!tuning)
                    George.shooter.setTargetVelocity(shooterVelocity);
                else
                    George.shooter.setTargetVelocity(tuneVelocity);
            }
            else {
                George.shooter.resetPID();
                George.shooter.setTargetVelocity(0);
                driveTurn = -gamepad1.right_stick_x;
            }

            targetHeadingEstimate = normalizeDegrees(Math.toDegrees(Math.atan2(localizerY, localizerX)))-180;
            distanceEstimate = Math.hypot(localizerX/39.37, localizerY/39.37);


            if (shooterButton.wasJustReleased()) {
                shooterOn = !shooterOn;
            }

            if (fieldOriented.wasJustReleased()) {
                isFieldOriented = !isFieldOriented;
            }

            if (resetHeading.wasJustReleased()) {
                George.localizer.resetHeading();
            }

            if (gamepad1.right_trigger>0) {
                George.shooter.setIntake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                George.shooter.setIntake(-0.55);
            }
            shooterButton.readValue();
            fieldOriented.readValue();
            resetHeading.readValue();
            reloacalize.readValue();
            George.drivetrain.drive(magnitude, theta, driveTurn, 0.875);
            George.shooter.updateShooter();
            George.shooter.setIntake(intakePower);
            telemetry.addData("Loc X: ", localizerX);
            telemetry.addData("Loc Y: ", localizerY);
            telemetry.addData("ShooterVelocity: ", shooterVelocity);
            telemetry.addData("Distance Estimate: ", distanceEstimate);
            telemetry.addData("Distance: ", distance);
            // FIX THESE REDUNDANT LOCALIZER CALLS!!!!!!!!!!!!!!!!
            telemetry.addData("Heading: ", George.localizer.getHeading());
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.addData("Target Heading Estimate: ", targetHeadingEstimate);
            telemetry.addData("Tx: ", tx);
            telemetry.addData("\nDriveTurn: ", driveTurn);
            telemetry.update();
        }
    }


    public double distanceFunction(double x, double theta) {  // Theta in degrees
        double xSquared = Math.pow(x, 2);
        double tanTheta = Math.tan(Math.toRadians(theta));   // Theta converted to radians before than
        return (xSquared)/((x*tanTheta)-1);
    }

    public double computeVelocity(double rawX) {
        // Haha Caveat
        if (rawX < 1.5) {
            return 128;
        }
        else if (rawX > 3.6) {
            return 170;
        }
        double x = distanceFunction(rawX, theta);
        double rawY = experimentalSlope*x;
        return Math.sqrt(rawY)*100;     // Velocity
    }

    private void getLocalizerValues() {
        localizerHeading = George.localizer.getHeading();
        localizerY = George.localizer.getPosY();
        localizerX = George.localizer.getPosX();
    }
}
