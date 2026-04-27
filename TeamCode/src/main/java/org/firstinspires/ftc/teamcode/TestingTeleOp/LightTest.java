package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Walt;

@TeleOp
public class LightTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Walt.init(hardwareMap, false, false);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                Walt.ledLights.blueColor();
            }
            if (gamepad1.bWasReleased()){
                Walt.ledLights.redColor();
            }
            if (gamepad1.xWasPressed()){
                Walt.ledLights.pinkColor();
            }
            if (gamepad1.yWasPressed()){
                Walt.ledLights.sparkle();
            }
        }
    }
}