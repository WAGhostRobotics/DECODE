package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zero Position" )
public class ServoTest extends OpMode {
    Servo s1;
    @Override
    public void init() {
        s1 = hardwareMap.get(Servo.class, "zero");
    }

    @Override
    public void loop () {
        if(gamepad1.dpad_up) {
            s1.setPosition(s1.getPosition() + 0.001);
        } else if(gamepad1.b){
            s1.setPosition(s1.getPosition() - 0.001);
        }

        telemetry.addData("Servo position", s1.getPosition());
        telemetry.update();
    }
}