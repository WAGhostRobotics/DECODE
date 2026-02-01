package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

//@TeleOp
//@Config
public class CameraTestRed extends LinearOpMode {
    public static int targetVelocity = 0;
    double rawX = 0, rawY = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Gus.init(hardwareMap, false, false);
        Gus.limelight.switchToBothGoalPipeline();
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
            Gus.localizer.update();
            Gus.limelight.trackAprilTag(Gus.localizer.getHeading(), Gus.shooter.getTurretAngle(), true);
            if (Gus.limelight.isVisible()) {
                Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
            }
            Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
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
