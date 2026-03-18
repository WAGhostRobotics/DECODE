package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
@Config
public class ShooterTuner extends OpMode {
    public static int targetVelocity = 0;
    public static double hoodPos = 0.5;
    public static double P = 0.01, I=0.00, D = 0, F = 0.003475, S = 0.02;

    @Override
    public void init() {
        Gus.init(hardwareMap, true, false);
        Gus.intake.openGate();
    }

    @Override
    public void loop() {
        Gus.shooter.setHood(hoodPos);
        if (gamepad1.right_bumper) {
            Gus.intake.shoot();     // No velocity control
        }
        else if (gamepad1.right_trigger > 0.1) {
            Gus.shooter.shoot();
        }
        else {
            Gus.intake.rollerStop();
            Gus.shooter.stop();
        }

        Gus.shooter.setPID(P, I, D, F, S);
        Gus.shooter.updateShooter();
        Gus.shooter.setTargetVelocity(targetVelocity);
        telemetry.addData("Shooter: ", Gus.shooter.getTelemetry());
        telemetry.addData("Voltage: ", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addLine(Gus.shooter.getVelocities());
        telemetry.update();
    }
}
