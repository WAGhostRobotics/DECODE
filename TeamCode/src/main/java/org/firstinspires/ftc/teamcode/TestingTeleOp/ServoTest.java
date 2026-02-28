package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zero Position" )
public class ServoTest extends OpMode {
    Servo s1;
    Servo s2;

    @Override

    public void init() {
        s1 = hardwareMap.get(Servo.class, "lift1");
        s2 = hardwareMap.get(Servo.class, "lift2");
        s1.setPosition(0);
        s2.setPosition(1);
    }

    @Override
    public void loop () {
        if(gamepad1.dpad_up) {
            s1.setPosition(s1.getPosition() + 0.001);
            s2.setPosition(1 - (s1.getPosition() + 0.001));
        } else if(gamepad1.b){
            s1.setPosition(s1.getPosition() - 0.001);
            s2.setPosition(1 - (s1.getPosition() - 0.001));
        }

        telemetry.addData("Lift1 position", s1.getPosition());
        telemetry.addData("Lift2 position", s2.getPosition());
        telemetry.update();
    }
}