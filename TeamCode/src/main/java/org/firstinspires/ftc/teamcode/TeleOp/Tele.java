package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

//@TeleOp
@Config
public class Tele extends LinearOpMode {

    double heading, turretAngle;
    ElapsedTime timer = new ElapsedTime();
    public static double intakePower = 0;
    public static double targetVelocity = 0;
    public static double hoodPos = 0.385;

    boolean shooterOn = false;

    int turretTargetPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader farShooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader shooterOff = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader headingReset = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);

        Bob.init(hardwareMap, true, true);
        Bob.limelight.switchToGoalPipeline();

        waitForStart();

        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.localizer.update();
            heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);


            Bob.limelight.trackAprilTag(normalizeDegrees(Bob.localizer.getHeading()-180), Bob.shooter.getTurretAngle(), true);

            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));

            if (Bob.limelight.isInitialized()) {
                Bob.shooter.updateTurret();
            }

            if (gamepad1.right_trigger>0.1) {
                Bob.intake.rollerIn();
            }
            else if (gamepad1.left_trigger>0.1) {
                Bob.intake.rollerOut();
            }
            else if (gamepad1.right_bumper) {
                Bob.shooter.shoot();
            }
            else {
                Bob.intake.rollerStop();
                Bob.shooter.stop();
            }

            if (shooterButton.wasJustReleased()) {
                shooterOn = true;
                targetVelocity = 200;
                hoodPos = 0.385;

            }
            else if (farShooterButton.wasJustReleased()) {
                shooterOn = true;
                hoodPos = 0.12;
                targetVelocity = 250;

            }
            else if (shooterOff.wasJustReleased()) {
                Bob.shooter.stop();
                Bob.shooter.setTurretTargetPos(0);
                shooterOn = false;
                targetVelocity = 0;
            }

            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.setHood(hoodPos);
            Bob.shooter.updateShooter();
            if (headingReset.wasJustReleased()) {
                Bob.localizer.resetHeading();
            }

            shooterButton.readValue();
            shooterOff.readValue();
            farShooterButton.readValue();
            headingReset.readValue();
            telemetry.addData("X:", Bob.localizer.getPosX());
            telemetry.addData("Y:", Bob.localizer.getPosY());
            telemetry.addData("Heading:", Bob.localizer.getHeading());
            telemetry.addData("Shooter: ", Bob.shooter.getTelemetry());
            telemetry.addData("Turret: ", Bob.shooter.getTurretTelemetry());
            telemetry.addData("Limelight: ", Bob.limelight.getTelemetry());
            telemetry.addData("Intake Current: ", Bob.intake.getCurrentDraw());
            telemetry.update();

        }
    }
}
