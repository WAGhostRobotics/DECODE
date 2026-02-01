package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class MainTeleOp extends LinearOpMode {
    public boolean blue = false;


    @Override
    public void runOpMode() throws InterruptedException {
        boolean full = false;
        boolean wasEmpty = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 1;
        ElapsedTime shootTimer, driveTimer;
        shootTimer = new ElapsedTime();
        driveTimer = new ElapsedTime();
        ToggleButtonReader zoneReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);



        waitForStart();
        Gus.init(hardwareMap, blue, true);
        Gus.localizer.setHeadingDegrees(180);
        Gus.intake.closeGate();

        while (opModeIsActive()) {
            // Remove later
//            Bob.limelight.setXYTranslation(xTranslation, yTranslation);


            if (failsafeButton.wasJustReleased()) {
                failsafe = !failsafe;
            }

            Gus.localizer.update();
            Gus.limelight.trackAprilTag(Gus.localizer.getHeading()-180, Gus.shooter.getTurretAngle(), true);
            double distance = Gus.limelight.getDistance();

            if (Gus.intake.isOneBallIn()) {
                if (wasEmpty) {
                    shootTimer.reset();
                    wasEmpty = false;
                }
                else if (shootTimer.seconds() > 0.5) {
                    Gus.intake.openGate();
                }


                if (!failsafe) {
                    Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
                }
                else {
                    Gus.shooter.setTurretTargetPos(0);
                }
            }
            else {
                shootTimer.reset();
            }


            Gus.shooter.updateTurret();
            if (Gus.intake.isFull() && !full) {
                full = true;
                gamepad1.rumble(200);
            }

            if (gateReader.wasJustReleased()) {
                full = false;
                wasEmpty = true;
                shooting = false;
                Gus.intake.setBallIn(false);
                Gus.intake.closeGate();
                Gus.shooter.resetTurret();
            }

            x = -gamepad1.left_stick_y;
            y = -gamepad1.left_stick_x;
            driveTurn = -gamepad1.right_stick_x;
            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            heading = Gus.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);

            if (Math.abs(magnitude) <= 0.1 && Math.abs(driveTurn) <= 0.1 && gamepad1.right_bumper ) {
                if (initialized && driveTimer.seconds() > 1) {
                    MotionPlanner.holdPosition(targetX, targetY, targetHeading);
                }
                else {
                    targetX = Gus.localizer.getPosX();
                    targetY = Gus.localizer.getPosY();
                    targetHeading = Gus.localizer.getHeading();
                }
            }
            else {
                initialized = true;
                Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
                driveTimer.reset();
            }


            Gus.intake.updateIntake();

            if (gamepad1.left_trigger>0.3) {
                Gus.intake.rollerOut();
            }
            else if (gamepad2.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    shooting = true;
                    Gus.shooter.resetTurret();
                    Gus.intake.loaderStop();
                    Gus.intake.rollerStop();
                    Gus.intake.setBallIn(true);
                    Gus.intake.openGate();
                }
                else if (shootTimer.seconds() > delay) {
                    Gus.shooter.shoot();
                }
                else {
                    Gus.intake.rollerStop();
                    Gus.shooter.stop();
                }
            }
            else if (shootButton.wasJustReleased()) {
                Gus.intake.loaderStop();
            }
            else {
                if (!shooting)
                    Gus.intake.rollerIn();
                else {
                    Gus.intake.loaderStop();
                    Gus.intake.rollerStop();
                }
            }

            if (!failsafe)
                Gus.shooter.setHood(Gus.shooterLUT.getHoodAngle(distance));
            else
                Gus.shooter.setHood(0.17);


            if (Gus.intake.gateOpen) {
                gamepad1.setLedColor(255, 0, 0, 5);
            }
            else {
                gamepad1.setLedColor(0, 255, 0, 5);
            }

            if (Gus.intake.isOneBallIn()) {
                if (!failsafe) {
                    Gus.shooter.setTargetVelocity(Gus.shooterLUT.getSpeed(distance));
                }
                else {
                    Gus.shooter.setTargetVelocity(182);
                }
            }
            else {
                Gus.shooter.setTargetVelocity(0);
            }
            Gus.shooter.updateShooter();

            gateReader.readValue();
            zoneReader.readValue();
            shootButton.readValue();
            imuReader.readValue();
            failsafeButton.readValue();


            if (imuReader.wasJustReleased()) {
                Gus.limelight.resetInitialized();
                Gus.localizer.setHeadingDegrees(180);
            }


//            telemetry.addData("Power: ", Gus.shooter.getTelemetry());
            telemetry.addData("Distance: ", Gus.limelight.getTelemetry());
//            telemetry.addData("X: ", Gus.localizer.getPosX());
//            telemetry.addData("Y: ", Gus.localizer.getPosY());
//            telemetry.addData("Heading: ", Gus.localizer.getHeading());
//            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
//            telemetry.addData("Timer: ", shootTimer.seconds());
            telemetry.update();

        }



    }
}
