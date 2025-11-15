package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HoodTesting extends LinearOpMode {
    public static double hoodPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo rightHood = hardwareMap.get(Servo.class, "rightHood");
        Servo leftHood = hardwareMap.get(Servo.class, "leftHood");
        while (opModeIsActive()) {
            rightHood.setPosition(hoodPos);
            leftHood.setPosition(1-hoodPos);

        }
    }
}
