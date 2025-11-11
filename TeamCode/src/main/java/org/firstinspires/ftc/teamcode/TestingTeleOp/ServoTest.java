package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Setting Position" )
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo = hardwareMap.get(Servo.class, "popper");


        servo.setPosition(0);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.a){
                servo.setPosition(servo.getPosition() + 0.001);
            } else if(gamepad1.b){
                servo.setPosition(servo.getPosition() - 0.001);
            }

            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();

        }
    }
}