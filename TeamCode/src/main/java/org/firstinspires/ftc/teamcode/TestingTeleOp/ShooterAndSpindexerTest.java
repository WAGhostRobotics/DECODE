package org.firstinspires.ftc.teamcode.TestingTeleOp;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class ShooterAndSpindexerTest extends LinearOpMode {
    public static int targetVelocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Bob.localizer.update();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            double turretAngle = Bob.shooter.getTurretAngle();

            double netAngle = turretAngle + heading;

            Bob.limelight.trackAprilTag(heading, turretAngle, true);
            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
            Bob.shooter.setHood(Shooter.hoodAngleToPos(Bob.limelight.getHoodAngle()));
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.updateShooter();

            if (gamepad1.right_trigger>0.2) {
                Bob.shooter.realShoot();
            }
            else {
                Bob.shooter.stop();
            }


            Bob.intake.updateSpindexer();
            telemetry.addData("Shooter: ", Bob.shooter.getTelemetry());
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.update();

        }
    }
}
