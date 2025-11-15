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
public class TeleRed extends LinearOpMode {

    double heading, turretAngle;
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

        Bob.init(hardwareMap, false, true);
        Bob.limelight.switchToGoalPipeline();

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.8);
            Bob.localizer.update();
            heading = Bob.localizer.getHeading();
            turretAngle = Bob.shooter.getTurretAngle();
            Bob.limelight.trackAprilTag(heading, turretAngle, true);


            if (gamepad1.right_bumper) {
                Bob.intake.rollerIn();
                Bob.shooter.rapidShoot();
            }
            else if (gamepad1.right_trigger>0.1) {
                Bob.intake.rollerIn();
                Bob.intake.autoIntake();
            }
            else {
                Bob.intake.holdAtZero();
                Bob.intake.rollerStop();
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



            if (shooterOn) {
                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
                Bob.shooter.updateTurret();

            }
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.setHood(hoodPos);
            Bob.shooter.updateShooter();
            Bob.localizer.update();
            Bob.intake.updateSpindexer();
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
