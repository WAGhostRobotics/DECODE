package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.teamcode.TestingTeleOp.AutoShootTuner.xTranslation;
import static org.firstinspires.ftc.teamcode.TestingTeleOp.AutoShootTuner.yTranslation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class CameraTest extends LinearOpMode {
    public static int targetVelocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Bob.init(hardwareMap);
        limelight3A.start();

        double heading = 0;
        double aprilX, aprilY, aprilXInches, aprilYInches, distance = 0;
        double distanceEstimate, targetHeading = 0;
        double shooterTarget = 0, normalizedShooterTarget= 0;
        boolean shooterOn = false;
        Pose3D botPose;
        double hoodAngle = 0, hoodPos = 0;



        waitForStart();
        while (opModeIsActive()) {
            Bob.localizer.update();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            double turretAngle = Bob.shooter.getTurretAngle();
            ;
            double netAngle = turretAngle + heading;


            if (gamepad1.dpad_left || gamepad1.right_trigger>0.1) {
                if (gamepad1.right_trigger > 0.1) {
                    Bob.shooter.shoot();
                }
                Bob.intake.rollerIn();
            }
            else if (gamepad1.dpad_right) {
                Bob.intake.rollerOut();
            }
            else {
                Bob.intake.rollerStop();
            }

            if (gamepad1.right_trigger < 0.1) {
                Bob.shooter.popDown();
            }





            limelight3A.updateRobotOrientation(netAngle);
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                botPose = llResult.getBotpose_MT2();
                aprilX = (botPose.getPosition().x + xTranslation);
                aprilY = (botPose.getPosition().y + yTranslation);
                aprilXInches = aprilX * 39.37;
                aprilYInches = aprilY * 39.37;
                distance = Math.hypot(aprilX, aprilY)*Math.cos(Math.toRadians(19));
                Bob.localizer.setPositionOnly(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, netAngle));
                targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(aprilYInches, aprilXInches)))-180;
                telemetry.addData("Pose: ", botPose);
            }
            else {
                double estimatedY = Bob.localizer.getPosY();
                double estimatedX = Bob.localizer.getPosX();
                distance = Math.hypot(estimatedX/39.37, estimatedY/39.37);
                targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(estimatedY, estimatedX))-180);
            }
            hoodAngle = getHoodAngle(distance);
            hoodPos = hoodAngleToPos(hoodAngle);
            shooterTarget = targetHeading - heading;
            normalizedShooterTarget = normalizeTurretAngle(shooterTarget);
            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(normalizedShooterTarget));


            if (shooterButton.wasJustReleased()) {
                shooterOn = true;
//                targetVelocity = 0;
//                Bob.shooter.setHood(0.2);
            }
            else if (farShooterButton.wasJustReleased()) {
                shooterOn = true;
//                Bob.shooter.setHood(0.05);
//                targetVelocity = 0;

            }
            else if (shooterOff.wasJustReleased()) {
                shooterOn = false;
//                targetVelocity = 0;

            }

            if (shooterOn) {
                Bob.shooter.updateTurret();
                Bob.shooter.setHood(hoodPos);
                Bob.shooter.setTargetVelocity(targetVelocity);

            }
            else {
                Bob.shooter.setTargetVelocity(0);
            }

            Bob.shooter.updateShooter();
            shooterButton.readValue();
            shooterOff.readValue();
            farShooterButton.readValue();
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.addData("Shooter Target: ", shooterTarget);
            telemetry.addData("Normalized Shooter Target: ", normalizedShooterTarget);
            telemetry.addData("Localizer: ", Bob.localizer.getHeading());
            telemetry.addData("X: ", Bob.localizer.getPosX());
            telemetry.addData("Y: ", Bob.localizer.getPosY());
            telemetry.addData("Turret Angle: ", turretAngle);
            telemetry.addData("Turret Tele: ", Bob.shooter.getTurretTelemetry());
            telemetry.addData("\nNet Angle: ", netAngle);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("Hood Angle: ", hoodAngle);
            telemetry.addData("Hood Pos: ", hoodPos);
            telemetry.update();
        }
    }

    private double normalizeTurretAngle(double degrees) {
        degrees = normalizeDegrees(degrees);
        if (degrees > 225)
            degrees = 225;
        else if (degrees < -45)
            degrees = -45;
        return degrees;
    }


    private double getHoodAngle(double distance) {
        return (90-Math.max(Math.min(Math.toDegrees(Math.atan(1.7/distance)), 65), 38));
    }

    private double hoodAngleToPos(double angle) {
        return -angle/54.0 + 52.0/54.0;
    }
}
