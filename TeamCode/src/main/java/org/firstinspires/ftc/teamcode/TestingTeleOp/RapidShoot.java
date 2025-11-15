package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class RapidShoot extends LinearOpMode {
    public static int targetVelocity = 0;
    public static double hoodPos = 0.3;
    public static double spinPower = 0.75;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.updateShooter();
            Bob.shooter.setHood(hoodPos);
            Bob.intake.setSpinPower(spinPower);

            if (gamepad1.right_trigger>0.2) {
                Bob.shooter.rapidShoot();
            }
            else {
                Bob.shooter.stop();
            }
        }
    }
}
