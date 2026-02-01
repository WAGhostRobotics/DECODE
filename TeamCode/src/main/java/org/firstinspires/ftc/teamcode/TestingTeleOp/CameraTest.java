package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

//@TeleOp
//@Config
public class CameraTest extends LinearOpMode {
    public static int targetVelocity = 0;
    boolean blue = true;
    public static double hoodPos = 0.5;
    public static double P = 0.00005, I = 0.000005, D = 0;
    public static double xTranslation = 1.5;
    public static double yTranslation = 1.5; // Y is only for Blue. Red would be negative
    double rawX = 0, rawY = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader zoneButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Gus.init(hardwareMap, true, false);
        limelight3A.start();

        double heading = 0;
        double aprilX, aprilY, aprilXInches, aprilYInches, distance = 0;
        double distanceEstimate, targetHeading = 0;
        double shooterTarget = 0, normalizedShooterTarget= 0;
        boolean shooterOn = false;
        Pose3D botPose;
        double hoodAngle = 0;

        Gus.limelight.switchToBothGoalPipeline();

        waitForStart();
        while (opModeIsActive()) {
            Gus.localizer.update();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            heading = Gus.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            double turretAngle = Gus.shooter.getTurretAngle();
            double netAngle = turretAngle + heading;


            if (gamepad1.dpad_left || gamepad1.right_trigger>0.1) {
                if (gamepad1.right_trigger > 0.1) {
                    Gus.shooter.shoot();
                }
                Gus.intake.rollerIn();
            }
            else if (gamepad1.dpad_right) {
                Gus.intake.rollerOut();
            }
            else {
                Gus.intake.rollerStop();
            }

            if (gamepad1.right_trigger < 0.1) {
                Gus.shooter.popDown();
            }





            limelight3A.updateRobotOrientation(netAngle);
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                botPose = llResult.getBotpose_MT2();
                rawX = botPose.getPosition().x;
                rawY = botPose.getPosition().y;
                aprilX = (botPose.getPosition().x + xTranslation);
                if (blue) {
                    aprilY = (botPose.getPosition().y + yTranslation);
                }
                else {
                    aprilY = (botPose.getPosition().y - yTranslation);
                }
                aprilXInches = aprilX * 39.37;
                aprilYInches = aprilY * 39.37;
                distance = Math.hypot(aprilX, aprilY) * Math.cos(Math.toRadians(19));
                Gus.localizer.setPositionOnly(new Pose2D(DistanceUnit.INCH, aprilXInches, aprilYInches, DEGREES, netAngle));
                targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(aprilYInches, aprilXInches))-180);
                telemetry.addData("Pose: ", botPose);
            }
            else {
                double estimatedY = Gus.localizer.getPosY();
                double estimatedX = Gus.localizer.getPosX();
                distance = Math.hypot(estimatedX/39.37, estimatedY/39.37) * Math.cos(Math.toRadians(19));
                targetHeading = normalizeDegrees(Math.toDegrees(Math.atan2(estimatedY, estimatedX))-180);
            }
            hoodAngle = getHoodAngle(distance);
//            hoodPos = hoodAngleToPos(hoodAngle);
            shooterTarget = targetHeading - heading;
            normalizedShooterTarget = normalizeTurretAngle(shooterTarget);
            Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(normalizedShooterTarget));


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
                Gus.shooter.updateTurret();
                Gus.shooter.setHood(hoodPos);
                Gus.shooter.setTargetVelocity(targetVelocity);

            }
            else {
                Gus.shooter.setTargetVelocity(0);
            }
            Gus.shooter.setTurretPID(P, I, D);

            Gus.shooter.updateShooter();
            shooterButton.readValue();
            shooterOff.readValue();
            farShooterButton.readValue();
            zoneButton.readValue();
            if (zoneButton.wasJustReleased()) {
                blue = !blue;
            }

            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.addData("Shooter Target: ", shooterTarget);
            telemetry.addData("Normalized Shooter Target: ", normalizedShooterTarget);
            telemetry.addData("Localizer: ", Gus.localizer.getHeading());
            telemetry.addData("RawX: " , rawX);
            telemetry.addData("RawY: ", rawY);
            telemetry.addData("X: ", Gus.localizer.getPosX());
            telemetry.addData("Y: ", Gus.localizer.getPosY());
            telemetry.addData("Turret Angle: ", turretAngle);
            telemetry.addData("Turret Tele: ", Gus.shooter.getTurretTelemetry());
            telemetry.addData("\nNet Angle: ", netAngle);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("Hood Angle: ", hoodAngle);
            telemetry.addData("Hood Pos: ", hoodPos);
            telemetry.addData("Shooter: ", Gus.shooter.getTelemetry());
            telemetry.addData("Blue: ", blue);
            telemetry.update();
        }
    }

    private double normalizeTurretAngle(double degrees) {
        return ((degrees + 90) % 360 ) - 90;
    }


    private double getHoodAngle(double distance) {
        return (90-Math.max(Math.min(Math.toDegrees(Math.atan(1.7/distance)), 65), 38));
    }

    private double hoodAngleToPos(double angle) {
        return ((angle-27.0)/36.0)*0.7;
    }
}
