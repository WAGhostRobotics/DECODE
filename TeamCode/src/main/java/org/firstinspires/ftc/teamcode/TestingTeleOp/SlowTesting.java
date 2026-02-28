package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
@Config
public class SlowTesting extends OpMode {
    public static double delayTime = 0.5;
    public static double shootTime = 0.5;
    public static double hoodPos = 0.5;
    public static int targetVelocity = 0;


    @Override
    public void init() {
        Gus.init(hardwareMap, false, false);
        Gus.intake.openGate();
    }

    @Override
    public void loop() {
        Gus.shooter.setDelayTime(delayTime);
        Gus.shooter.setShootTime(shootTime);
        Gus.shooter.setHood(hoodPos);
        Gus.shooter.setTargetVelocity(targetVelocity);
        Gus.shooter.updateShooter();

        if (gamepad1.right_bumper) {
            Gus.shooter.shootSlowMotion();
        }
        else {
            Gus.shooter.stop();
            Gus.intake.shootStop();
            Gus.intake.rollerStop();
        }

        telemetry.addData("Shooter: ", Gus.shooter.getTelemetry());


    }
}
