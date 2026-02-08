package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;


@Config
@TeleOp
public class Interpolater extends LinearOpMode {
    boolean failsafe = false;
    public static double delay = 1;
    ElapsedTime timer;

    public static double xTranslation = 1.7;
    public static double yTranslation = 1.3;
    public static double hoodPos = 0.5;
    public static int targetVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        ToggleButtonReader zoneReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);



        waitForStart();
        Gus.init(hardwareMap, false, false);
        Gus.localizer.setHeadingDegrees(180);
        Gus.intake.closeGate();

        while (opModeIsActive()) {
            // Remove later
            Gus.limelight.setXYTranslation(xTranslation, yTranslation);


            if (failsafeButton.wasJustReleased()) {
                failsafe = !failsafe;
            }

            Gus.localizer.update();
            Gus.limelight.trackAprilTag(Gus.localizer.getHeading()-180, Gus.shooter.getTurretAngle(), false);
            double distance = Gus.limelight.getDistance();

            if (Gus.intake.isOneBallIn()) {
                if (!failsafe) {
                    Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
                }
                else {
                    Gus.shooter.setTurretTargetPos(0);
                }
            }
            Gus.shooter.updateTurret();


            if (gateReader.wasJustReleased()) {
                Gus.intake.setBallIn(false);
                Gus.intake.closeGate();
                Gus.shooter.resetTurret();
            }

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double heading = Gus.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);
            Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
            Gus.intake.updateIntake();

            if (gamepad1.left_trigger>0.3) {
                Gus.intake.rollerOut();
            }
            else if (gamepad2.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    Gus.shooter.resetTurret();
                    timer.reset();
                    Gus.intake.loaderStop();
                    Gus.intake.rollerStop();
                    Gus.intake.setBallIn(true);
                    Gus.intake.openGate();
                }
                else if (timer.seconds() > delay) {
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
                timer.reset();
                if (!Gus.intake.gateOpen)
                    Gus.intake.rollerIn();
                else {
                    Gus.intake.loaderStop();
                    Gus.intake.rollerStop();
                }
            }

            Gus.shooter.setHood(hoodPos);


            if (Gus.intake.gateOpen) {
                gamepad2.setLedColor(255, 0, 0, 5);
            }
            else {
                gamepad2.setLedColor(0, 255, 0, 5);
            }

            if (Gus.intake.isOneBallIn()) {
                Gus.shooter.setTargetVelocity(targetVelocity);
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


            telemetry.addData("Power: ", Gus.shooter.getTelemetry());
            telemetry.addData("Distance: ", Gus.limelight.getDistance());
            telemetry.addData("X: ", Gus.localizer.getPosX());
            telemetry.addData("Y: ", Gus.localizer.getPosY());
            telemetry.addData("Heading: ", Gus.localizer.getHeading());
            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();

        }



    }
}
