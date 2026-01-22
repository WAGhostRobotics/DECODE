package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class LiftTest extends OpMode {
    Servo lift1;
    Servo lift2;
    double pos = 1;
    @Override
    public void init() {
        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            pos = Math.min(1, pos+0.001);
        }
        else if (gamepad1.b) {
            pos = Math.max(0, pos-0.001);
        }

        lift1.setPosition(pos);
        lift2.setPosition(1-pos);
        telemetry.addData("Pos: ", pos);
        telemetry.update();
    }
}
