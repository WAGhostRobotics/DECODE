package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class TurretTuner extends LinearOpMode {
    public static double P=0.0002, I=0.00007, D;
    public static int permissible = 50;
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        Bob.limelight.switchToGoalPipeline();
        waitForStart();
        while (opModeIsActive()) {
            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading(), Bob.shooter.getTurretAngle(), true);
            Bob.shooter.setTurretPID(P, I, D);
            if (gamepad1.a) {
                targetPosition += 10;
            }
            else if (gamepad1.b) {
                targetPosition -= 10;
            }
            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
            Bob.shooter.updateTurret();
            telemetry.addData("Turret: ", Bob.shooter.getTurretTelemetry());
//            telemetry.addData("error: ", error);
            telemetry.update();
        }
    }
}
