package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
public class LightTest extends LinearOpMode {
    boolean red = true;
    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, false, false);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                red = true;
            }
            else if (gamepad1.b) {
                red = false;
            }
            if (red) {
                Gus.ledLights.orange();
            }
            else {
                Gus.ledLights.green();
            }
        }
    }
}
