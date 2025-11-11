package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "lf");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                m1.setPower(0.5);
            }
            else if (gamepad1.b) {
                m1.setPower(-0.5);
            }
            else {
                m1.setPower(0);
            }
        }
    }
}
