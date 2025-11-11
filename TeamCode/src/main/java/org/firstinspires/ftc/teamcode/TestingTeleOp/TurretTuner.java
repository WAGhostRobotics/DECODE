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
    PIDController turretController = new PIDController(0, 0, 0);
    public static double P=0.00008, I=0.00009, D;
    public static int permissible = 50;
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter();
        shooter.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                targetPosition += 10;
            }
            else if (gamepad1.b) {
                targetPosition -= 10;
            }
            shooter.setTurretTargetPos(targetPosition);
            shooter.updateTurret();
            telemetry.addData("Turret: ", shooter.getTurretTelemetry());
//            telemetry.addData("error: ", error);
            telemetry.update();
        }
    }
}
