package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Setting Position" )
public class ServoTest extends OpMode {
    long lastTime = System.nanoTime();
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "popper");
        servo.setPosition(0);
        telemetry.setMsTransmissionInterval(200);
    }

    @Override
    public void loop () {
        if(gamepad1.a) {
            servo.setPosition(servo.getPosition() + 0.001);
        } else if(gamepad1.b){
            servo.setPosition(servo.getPosition() - 0.001);
        }

        telemetry.addData("servo position", servo.getPosition());
        telemetry.addData("Loop Speed: ", calculateLoopSpeed());
        telemetry.update();
    }
    private double calculateLoopSpeed() {
        long now = System.nanoTime();
        double dt = (now-lastTime) / 1e9;
        lastTime = now;
        return 1.0/dt;
    }
}