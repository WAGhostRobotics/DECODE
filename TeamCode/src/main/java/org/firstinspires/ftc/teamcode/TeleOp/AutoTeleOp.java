package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Components.RI3W.Constants;
import org.firstinspires.ftc.teamcode.RI3W.George;

@TeleOp
@Config
public class AutoTeleOp extends LinearOpMode {

    boolean shooterOn = false;
    boolean locked = true;
    boolean isFieldOriented = true;
    double magnitude, lastMagnitude, x, y, heading, theta, driveTurn;
    public static double shooterVelocity;

    Pose2D targetPose, currentPose;
    boolean justStopped = false;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader fieldOriented = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader resetHeading = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader reloacalize = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        George.init(hardwareMap);

        while (opModeInInit()) {
            George.limelight.getNewHeading();
        }

        waitForStart();
        while (opModeIsActive()) {
            George.localizer.update();
            setLocalizerValues();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            driveTurn = -gamepad1.right_stick_x;

            if (isFieldOriented) {
                theta = normalizeDegrees(theta - George.localizer.getHeading());
            }


            checkJustStopped();
            George.limelight.trackAprilTag(heading, !locked);

            if (shooterOn) {
                shooterVelocity = George.limelight.getShooterVelocity();
                George.shooter.setTargetVelocity(shooterVelocity);
                if (George.shooter.reachedVelocity()) {
                    gamepad1.setLedColor(0, 255, 0, 100);
                }
                else {
                    gamepad1.setLedColor(255, 0, 0, 100);
                }
                driveTurn = George.limelight.getDriveTurn();
            }
            else {
                George.shooter.resetPID();
//                George.shooter.standBy();
                George.shooter.stop();
                gamepad1.setLedColor(0, 0, 255, 100);
            }

            if (Double.isNaN(driveTurn)) {
                driveTurn = 0;
                George.limelight.resetHeadingControl();
            }


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
                if (shooterOn)
                    George.shooter.shoot();
                else
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

            driveAndHold();
            George.shooter.updateShooter();
//            telemetry.addData("", George.limelight.getTelemetry());
            telemetry.addData("\nShooter: ", George.shooter.getTelemetry());
//            telemetry.addData("Magnitude: ", magnitude);
//            telemetry.addData("JustStopped: ", justStopped);
//            telemetry.addData("Timer: ", timer.seconds());
//            telemetry.addData("Locked: ", locked);
//            telemetry.addData("TargetPose: ", targetPose);

            telemetry.update();
        }
    }

    private void checkJustStopped() {
        if (!shooterOn) {
            lastMagnitude = 1;
            justStopped = false;
            return;
        }
        if (Math.abs(magnitude) < 0.01 && Math.abs(lastMagnitude) > 0.01) {
            justStopped = true;
            timer.reset();
        }
        else if (magnitude != 0) {
            justStopped = false;
        }
        lastMagnitude = magnitude;
    }

    private void setLocalizerValues() {
        x = George.localizer.getPosX();
        y = George.localizer.getPosY();
        heading = George.localizer.getHeading();
    }

    private void driveAndHold() {
        if (shooterOn && justStopped && Math.abs(magnitude) <= 0.01) {
            currentPose = new Pose2D(DistanceUnit.INCH, x, y, DEGREES, heading);
            if (timer.seconds()>0.3) {
                locked = true;
                George.drivetrain.holdChassis(currentPose, targetPose, driveTurn);
            }
            else {
                locked = false;
                targetPose = currentPose;
                George.drivetrain.drive(0,0,driveTurn, 0);
            }
        }
        else {
            locked = false;
            George.drivetrain.drive(magnitude, theta, driveTurn, 0.95);
        }
    }
}
