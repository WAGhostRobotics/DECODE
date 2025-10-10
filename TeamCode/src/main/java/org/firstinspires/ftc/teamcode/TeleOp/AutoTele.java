package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RI3W.George;

@TeleOp
public class AutoTele extends LinearOpMode {

    boolean shooterOn = false;
    boolean isFieldOriented = true;

    @Override
    public void runOpMode() throws InterruptedException {
        double lastAprilHeading = 0;
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader fieldOriented = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader resetHeading = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader reloacalize = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        George.init(hardwareMap);

        while (opModeInInit()) {
            George.limelight.trackAprilTag();
        }

        waitForStart();
        while (opModeIsActive()) {
            George.localizer.update();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double driveTurn = -gamepad1.right_stick_x;
            if (isFieldOriented) {
                theta = normalizeDegrees(theta - George.localizer.getHeading());
            }

            if (shooterOn) {
                George.limelight.trackAprilTag();
                double shooterVelocity = George.limelight.getShooterVelocity();
                George.shooter.setTargetVelocity(shooterVelocity);
                driveTurn = George.limelight.getDriveTurn();
                lastAprilHeading = George.limelight.getAprilHeading();


            }
            else {
                George.shooter.resetPID();
                George.shooter.setTargetVelocity(0);
            }


            if (shooterButton.wasJustReleased()) {
                if (shooterOn) {
                    double newHeading = normalizeDegrees(lastAprilHeading+90);
                    George.localizer.setHeadingDegrees(newHeading);
                }
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
            else {
                George.shooter.setIntake(0);
            }
            shooterButton.readValue();
            fieldOriented.readValue();
            resetHeading.readValue();
            reloacalize.readValue();
            George.drivetrain.drive(magnitude, theta, driveTurn, 0.875);
            George.shooter.updateShooter();
            telemetry.addData("", George.limelight.getTelemetry());
            telemetry.update();
        }
    }
}
