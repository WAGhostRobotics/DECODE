package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Walt;

@TeleOp
@Config
public class ShooterTuner extends OpMode {
    public static int targetVelocity = 0;
    public static double hoodPos = 0.6;
    public static double P = 0.01, I=0.00, D = 0, F = 0.003475, S = 0.02;

    @Override
    public void init() {
        Walt.init(hardwareMap, true, false);
        Walt.intake.openGate();
    }

    @Override
    public void loop() {
        Walt.shooter.setHood(hoodPos);
        if (gamepad1.right_bumper) {
            Walt.intake.shoot();     // No velocity control
        }
        else if (gamepad1.right_trigger > 0.1) {
            Walt.shooter.shoot();
        }
        else {
            Walt.intake.rollerStop();
            Walt.shooter.stop();
        }

        Walt.shooter.setPID(P, I, D, F, S);
        Walt.shooter.updateShooter();
        Walt.shooter.setTargetVelocity(targetVelocity);
        telemetry.addData("Shooter: ", Walt.shooter.getTelemetry());
        telemetry.addData("Voltage: ", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addLine(Walt.shooter.getVelocities());
        telemetry.update();
    }
}
