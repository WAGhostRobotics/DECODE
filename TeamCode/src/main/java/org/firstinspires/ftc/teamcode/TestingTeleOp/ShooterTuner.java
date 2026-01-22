package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class ShooterTuner extends OpMode {
    public static int targetVelocity = 0;
    public static double P = 0.12, I = 0.0055, D = 0;

    @Override
    public void init() {
        Bob.init(hardwareMap, true, false);
        Bob.intake.openGate();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            Bob.shooter.shoot();
        }
        else {
            Bob.intake.rollerStop();;
            Bob.shooter.stop();
        }
        Bob.shooter.setPID(P, I, D);
        Bob.shooter.updateShooter();
        Bob.shooter.setTargetVelocity(targetVelocity);
        telemetry.addData("Shooter: ", Bob.shooter.getTelemetry());
        telemetry.update();
    }
}
