package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
@Config
public class AirSort extends OpMode {
    public static double arcHoodPos = 0.8;
    public static double straightHoodPos = 0.2;
    public static double hoodPos = 0.5;
    public static double time = 0.5;
    public static int targetVelocity = 0;
    public static int straightV = 168;
    public static int arcV = 160;

    public static boolean shooting = false;

    ElapsedTime timer;
    @Override
    public void init() {
        timer = new ElapsedTime();
        Gus.init(hardwareMap, false, false);
        Gus.intake.openGate();
    }

    @Override
    public void loop() {
        if (shooting) {
            if (timer.seconds() > time) {
                Gus.shooter.setHood(straightHoodPos);
                Gus.shooter.setTargetVelocity(straightV);
            }
            else {
                Gus.shooter.setHood(arcHoodPos);
                Gus.shooter.setTargetVelocity(arcV);
            }
        }
        else {
            Gus.shooter.setHood(hoodPos);
            Gus.shooter.setTargetVelocity(targetVelocity);
            timer.reset();
        }

        if (gamepad1.right_bumper) {
            shooting = true;
            Gus.intake.shoot();
        }
        else {
            shooting = false;
            Gus.intake.shootStop();
            Gus.intake.rollerStop();
        }

        Gus.shooter.updateShooter();

    }
}
