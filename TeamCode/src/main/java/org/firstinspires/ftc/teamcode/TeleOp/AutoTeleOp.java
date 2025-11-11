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
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class AutoTeleOp extends LinearOpMode {

    boolean shooterOn = false;
    boolean locked = true;
    boolean isFieldOriented = true;
    double magnitude, lastMagnitude, x, y, heading, theta, driveTurn;
    public static double shooterVelocity, turretAngle, hoodPos;

    Pose2D targetPose, currentPose;
    boolean justStopped = false;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader fieldOriented = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader resetHeading = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader reloacalize = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        Bob.init(hardwareMap);

        while (opModeInInit()) {
            Bob.limelight.getNewHeading();
        }

        waitForStart();
        while (opModeIsActive()) {
            Bob.localizer.update();
            setLocalizerValues();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            driveTurn = -gamepad1.right_stick_x;

            if (isFieldOriented) {
                theta = normalizeDegrees(theta - Bob.localizer.getHeading());
            }


            checkJustStopped();
            Bob.limelight.trackAprilTag(heading, Bob.shooter.getTurretAngle(), !locked);

            if (shooterOn) {
                shooterVelocity = Bob.limelight.getShooterVelocity();
                turretAngle = Bob.limelight.getTurretAngle();
                hoodPos = Bob.limelight.getHoodPos();
                Bob.shooter.setTargetVelocity(shooterVelocity);
                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
                Bob.shooter.updateTurret();
                Bob.shooter.setHood(hoodPos);
                if (Bob.shooter.reachedVelocity()) {
                    gamepad1.setLedColor(0, 255, 0, 100);
                }
                else {
                    gamepad1.setLedColor(255, 0, 0, 100);
                }
                turretAngle = Bob.limelight.getTurretAngle();
            }
            else {
                Bob.shooter.resetPID();
//                George.shooter.standBy();
                Bob.shooter.stop();
                gamepad1.setLedColor(0, 0, 255, 100);
            }


            if (shooterButton.wasJustReleased()) {
                shooterOn = !shooterOn;
            }

            if (fieldOriented.wasJustReleased()) {
                isFieldOriented = !isFieldOriented;
            }

            if (resetHeading.wasJustReleased()) {
                Bob.localizer.resetHeading();
            }

            if (gamepad1.right_trigger>0) {
                if (shooterOn)
                    Bob.shooter.shoot();
                else
                    Bob.shooter.setIntake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                Bob.shooter.setIntake(-0.55);
            }
            else {
                Bob.shooter.setIntake(0);
            }
            shooterButton.readValue();
            fieldOriented.readValue();
            resetHeading.readValue();
            reloacalize.readValue();

            driveAndHold();
            Bob.shooter.updateShooter();

//            telemetry.addData("", George.limelight.getTelemetry());
            telemetry.addData("\nShooter: ", Bob.shooter.getTelemetry());
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
        x = Bob.localizer.getPosX();
        y = Bob.localizer.getPosY();
        heading = Bob.localizer.getHeading();
    }

    private void driveAndHold() {
        if (shooterOn && justStopped && Math.abs(magnitude) <= 0.01) {
            currentPose = new Pose2D(DistanceUnit.INCH, x, y, DEGREES, heading);
            if (timer.seconds()>0.3) {
                locked = true;
                Bob.drivetrain.holdChassis(currentPose, targetPose, 0);
            }
            else {
                locked = false;
                targetPose = currentPose;
                Bob.drivetrain.drive(0,0, driveTurn, 0);
            }
        }
        else {
            locked = false;
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.95);
        }
    }
}
