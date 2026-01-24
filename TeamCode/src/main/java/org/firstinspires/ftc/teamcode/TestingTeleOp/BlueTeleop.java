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
import org.firstinspires.ftc.teamcode.Core.Bob;


@Config
@TeleOp
public class BlueTeleop extends LinearOpMode {
    boolean blue = true;
    boolean failsafe = false;
    public static double delay = 1;
    boolean shooterOn = false;
    ElapsedTime timer;

    public static double xTranslation = 1.55;
    public static double yTranslation = 1.3;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        ToggleButtonReader zoneReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);



        waitForStart();
        Bob.init(hardwareMap, true, true);
        Bob.localizer.setHeadingDegrees(180);
        Bob.intake.closeGate();

        while (opModeIsActive()) {
            // Remove later
//            Bob.limelight.setXYTranslation(xTranslation, yTranslation);


            if (failsafeButton.wasJustReleased()) {
                failsafe = !failsafe;
            }

            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading()-180, Bob.shooter.getTurretAngle(), true);
            double distance = Bob.limelight.getDistance();

            if (Bob.intake.isOneBallIn()) {
                if (!failsafe) {
                    Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
                }
                else {
                    Bob.shooter.setTurretTargetPos(0);
                }
            }
            Bob.shooter.updateTurret();


            if (gateReader.wasJustReleased()) {
                Bob.intake.setBallIn(false);
                Bob.intake.closeGate();
                Bob.shooter.resetTurret();
            }

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double heading = Bob.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
            Bob.intake.updateIntake();

            if (gamepad1.left_trigger>0.3) {
                Bob.intake.rollerOut();
            }
            else if (gamepad2.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    timer.reset();
                    Bob.intake.loaderStop();
                    Bob.intake.rollerStop();
                    Bob.intake.setBallIn(true);
                    Bob.intake.openGate();
                }
                else if (timer.seconds() > delay) {
                    Bob.shooter.shoot();
                }
                else {
                    Bob.intake.rollerStop();
                    Bob.shooter.stop();
                }
            }
            else if (shootButton.wasJustReleased()) {
                Bob.intake.loaderStop();
            }
            else {
                timer.reset();
                if (!Bob.intake.gateOpen)
                    Bob.intake.rollerIn();
                else {
                    Bob.intake.loaderStop();
                    Bob.intake.rollerStop();
                }
            }

            if (!failsafe)
                Bob.shooter.setHood(Bob.shooterLUT.getHoodAngle(distance));
            else
                Bob.shooter.setHood(0.17);


            if (Bob.intake.gateOpen) {
                gamepad2.setLedColor(255, 0, 0, 5);
            }
            else {
                gamepad2.setLedColor(0, 255, 0, 5);
            }

            if (Bob.intake.isOneBallIn()) {
                if (!failsafe) {
                    Bob.shooter.setTargetVelocity(Bob.shooterLUT.getSpeed(distance));
                }
                else {
                    Bob.shooter.setTargetVelocity(182);
                }
            }
            else {
                Bob.shooter.setTargetVelocity(0);
            }
            Bob.shooter.updateShooter();

            gateReader.readValue();
            zoneReader.readValue();
            shootButton.readValue();
            imuReader.readValue();
            failsafeButton.readValue();


            if (imuReader.wasJustReleased()) {
                Bob.limelight.resetInitialized();
                Bob.localizer.setHeadingDegrees(180);
            }


            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Distance: ", Bob.limelight.getDistance());
            telemetry.addData("X: ", Bob.localizer.getPosX());
            telemetry.addData("Y: ", Bob.localizer.getPosY());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();

        }



    }
}
