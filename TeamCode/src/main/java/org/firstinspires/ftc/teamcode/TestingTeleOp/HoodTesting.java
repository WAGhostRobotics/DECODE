package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HoodTesting extends OpMode {
    public static double hoodPos = 0;
    Servo rightHood;

    @Override
    public void init(){
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        rightHood.setPosition(hoodPos);
    }

    @Override
    public void loop() {
            if (gamepad1.a) {
                hoodPos = Math.min(hoodPos + 0.001, 1);
            }
            else if (gamepad1.b) {
                hoodPos = Math.max(hoodPos - 0.001, 0);
            }
            rightHood.setPosition(hoodPos);

            telemetry.addData("Hood Pos: ", hoodPos);
            telemetry.update();
    }

    @Override
    public void stop() {
    }
}
