package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class Tele extends LinearOpMode {
    public static double intakePower = 0;
    public static double targetVelocity = 0;
    public static double hoodPos = 0;

    boolean shooterOn = false;

    int turretTargetPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);

        Bob.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

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

            if (shooterButton.wasJustReleased()) {
                shooterOn = true;
                targetVelocity = 110;
                hoodPos = 0.2;

            }
            else if (farShooterButton.wasJustReleased()) {
                shooterOn = true;
                hoodPos = 0.05;
                targetVelocity = 140;

            }
            else if (shooterOff.wasJustReleased()) {
                shooterOn = false;
                targetVelocity = 0;

            }

            if (gamepad1.right_bumper) {
                turretTargetPos -= 35;
            }
            else if (gamepad1.left_bumper) {
                turretTargetPos += 35;
            }

            if (shooterOn) {
                Bob.shooter.setTurretTargetPos(turretTargetPos);
                Bob.shooter.updateTurret();

            }
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.setHood(hoodPos);
            Bob.shooter.updateShooter();
            Bob.localizer.update();
            Bob.localizer.setHeadingDegrees(Bob.localizer.getHeading());
            shooterButton.readValue();
            shooterOff.readValue();
            farShooterButton.readValue();
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("X: ", Bob.localizer.getPosX());
            telemetry.addData("Y: ", Bob.localizer.getPosY());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("theta: ", theta);
            telemetry.addData("magnitude: ", magnitude);
            telemetry.addData("JoyStick X: ", x);
            telemetry.addData("JoyStick Y: ", y);
            telemetry.addData("Shooter: ", Bob.shooter.getTelemetry());
            telemetry.update();

        }
    }
}
