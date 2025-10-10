package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.RI3W.Shooter;
import org.firstinspires.ftc.teamcode.RI3W.George;

@TeleOp
@Config
public class Tele extends LinearOpMode {
    public static double intakePower = 0;
    public static double targetVelocity = 0;
    public static double servoPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);

        George.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            George.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            if (gamepad1.right_bumper) {
                intakePower = 1;
            }
            else if (gamepad1.left_bumper) {
                intakePower = -0.6;
            }
            else {
                intakePower = 0;
            }

            if (shooterButton.wasJustReleased()) {
                targetVelocity = Shooter.shootSpeed;
            }
            if (farShooterButton.wasJustReleased()) {
                targetVelocity = Shooter.farShootSpeed;
            }
            if (shooterOff.wasJustReleased()) {
                targetVelocity = 0;
            }

            George.shooter.setIntake(intakePower);
            George.shooter.setTargetVelocity(targetVelocity);
            George.shooter.updateShooter();
            George.shooter.setIntake(intakePower);
            George.localizer.update();

            shooterButton.readValue();
            shooterOff.readValue();
            farShooterButton.readValue();
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("X: ", George.localizer.getPosX());
            telemetry.addData("Y: ", George.localizer.getPosY());
            telemetry.addData("Heading: ", George.localizer.getHeading());
            telemetry.addData("theta: ", theta);
            telemetry.addData("magnitude: ", magnitude);
            telemetry.addData("JoyStick X: ", x);
            telemetry.addData("JoyStick Y: ", y);
            telemetry.update();

        }
    }
}
