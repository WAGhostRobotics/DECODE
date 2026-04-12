package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    public static double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.get(Servo.class, "rightHood");
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            hood.setPosition(pos);
            if (gamepad1.a) {
                m1.setPower(0.5);
                m2.setPower(0.5);
            }
            else if (gamepad1.b) {
                m1.setPower(-0.5);
                m2.setPower(-0.5);
            }
            else {
                m1.setPower(0);
                m2.setPower(0);
            }

            telemetry.addData("M1: ", m1.getVelocity());
            telemetry.addData("M2: ", m2.getVelocity());
            telemetry.update();
        }
    }
}
