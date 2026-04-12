package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.LedLights;
import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
public class LightTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, false, false);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                Gus.ledLights.blueColor();
            }
            if (gamepad1.bWasReleased()){
                Gus.ledLights.redColor();
            }
            if (gamepad1.xWasPressed()){
                Gus.ledLights.pinkColor();
            }
            if (gamepad1.yWasPressed()){
                Gus.ledLights.sparkle();
            }
        }
    }
}