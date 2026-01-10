package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.ShooterLUT;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    boolean blue = true;
    public static double intakePower = 1;                 // Change this in dashboard at runtime
    public static double spinPower = 0.75;
    public static double targetVelocity = 0;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public static double P = 0.125, I=0.00275, D = 0;
    public static double hoodPos = 0;

    public double currentVelocity, error;



    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shootReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader zoneReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        Bob.init(hardwareMap, false, false);

        while (opModeInInit()) {
            zoneReader.readValue();
            if (zoneReader.wasJustReleased()) {
                blue = !blue;
            }
            telemetry.addData("Blue: ", blue);
            telemetry.update();
        }
        waitForStart();
        Bob.intake.closeGate();
        Bob.limelight.setBlueAlliance(blue);
        Bob.limelight.switchToGoalPipeline();
        while (opModeIsActive()) {
            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading(), Bob.shooter.getTurretAngle(), true);
            double distance = Bob.limelight.getDistance();
            if (Bob.limelight.isVisible()) {
                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
            }
            Bob.shooter.updateTurret();
            if (shootReader.wasJustReleased()) {
                if (Bob.intake.gateOpen) {
                    Bob.intake.closeGate();
                }
                else {
                    Bob.intake.openGate();
                }
            }

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);


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

            Bob.shooter.setHood(Bob.shooterLUT.getHoodAngle(distance));
            if (Bob.intake.gateOpen) {
                Bob.shooter.setTargetVelocity(Bob.shooterLUT.getSpeed(distance));
            }
            else {
                Bob.shooter.setTargetVelocity(0);
            }
            Bob.shooter.updateShooter();

            shootReader.readValue();
            zoneReader.readValue();
            imuReader.readValue();

            if (zoneReader.wasJustReleased()) {
                blue = !blue;
                Bob.limelight.setBlueAlliance(blue);
                Bob.limelight.switchToGoalPipeline();
            }

            if (imuReader.wasJustReleased()) {
                Bob.localizer.resetHeading();
            }

            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("Blue: ", blue);
            telemetry.update();

        }



    }
    private void rollerIn() {

    }


}
