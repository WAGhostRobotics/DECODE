package org.firstinspires.ftc.teamcode.TestingTeleOp.RI3W;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class motorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, "motor");

        if (gamepad1.a) {
            m.setPower(1);
        } else if(gamepad1.b){
            m.setPower(-1);
        } else{
            m.setPower(0);
        }
    }
}
