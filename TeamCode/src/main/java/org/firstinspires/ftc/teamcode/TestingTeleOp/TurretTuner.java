package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
@Config
public class TurretTuner extends LinearOpMode {
    public static double P=0.000013, I=0.0000005, D;
    public static double turretKStatic = 0.04;
    public static int permissible = 50;
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, false, false);
        ToggleButtonReader switchReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        Gus.limelight.switchToGoalPipeline();
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Gus.localizer.update();
            double heading = Gus.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.9);

            Gus.localizer.update();
            Gus.limelight.trackAprilTag(Gus.localizer.getHeading(), Gus.shooter.getTurretAngle(), true);
            Gus.shooter.setTurretKStatic(turretKStatic);
            Gus.shooter.setTurretPID(P, I, D);

            if (switchReader.wasJustReleased()) {
                Gus.limelight.switchToGoalPipeline();
            }

            if (gamepad1.a) {
                targetPosition += 10;
            }
            else if (gamepad1.b) {
                targetPosition -= 10;
            }
            if (Gus.limelight.isVisible()) {
//                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
            }
            Gus.shooter.setTurretTargetPos(targetPosition);

            Gus.shooter.updateTurret();
            switchReader.readValue();
            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
            telemetry.addData("Position: ", Gus.shooter.getPosition());
//            telemetry.addData("error: ", error);
            telemetry.addData("Camera: ", Gus.limelight.getTelemetry());
            telemetry.update();
        }
    }
}

