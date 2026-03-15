package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "wheel2");
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            telemetry.addData("Wheel1:", m1.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Wheel1:", m1.getCurrentPosition());
            telemetry.update();
        }
    }
}
