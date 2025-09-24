package org.firstinspires.ftc.teamcode.RI3W;

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

        Servo servo = hardwareMap.get(Servo.class, "ball stopper");
        ButtonReader A = new ButtonReader(
                new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ButtonReader B = new ButtonReader(
                new GamepadEx(gamepad1), GamepadKeys.Button.B);

        servo.setPosition(0);

        waitForStart();

        while(opModeIsActive()) {

            if(A.wasJustReleased()){
                servo.setPosition(servo.getPosition() + 0.001);
            } else if(B.wasJustReleased()){
                servo.setPosition(servo.getPosition() - 0.001);
            }

            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();

        }
    }
}