package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

//@TeleOp
@Config
public class TurretTuner extends LinearOpMode {
    public static double P=0.0002, I=0.00007, D;
    public static int permissible = 50;
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap, false, false);
        ToggleButtonReader switchReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        Bob.limelight.switchToGoalPipeline();
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.localizer.update();
            double heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);

            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading(), Bob.shooter.getTurretAngle(), true);
            Bob.shooter.setTurretPID(P, I, D);
            if (switchReader.wasJustReleased()) {
                Bob.limelight.switchToGoalPipeline();
            }

            if (gamepad1.a) {
                targetPosition += 10;
            }
            else if (gamepad1.b) {
                targetPosition -= 10;
            }
            if (Bob.limelight.isVisible()) {
//                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
            }
            Bob.shooter.setTurretTargetPos(targetPosition);

            Bob.shooter.updateTurret();
            switchReader.readValue();
            telemetry.addData("Turret: ", Bob.shooter.getTurretTelemetry());
            telemetry.addData("Position: ", Bob.shooter.getPosition());
//            telemetry.addData("error: ", error);
            telemetry.addData("Camera: ", Bob.limelight.getTelemetry());
            telemetry.update();
        }
    }
}

