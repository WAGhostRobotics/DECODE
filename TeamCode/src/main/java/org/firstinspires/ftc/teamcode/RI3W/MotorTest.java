package org.firstinspires.ftc.teamcode.RI3W;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    public static double power = 0;                 // Change this in dashboard at runtime
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        DcMotorEx wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                power += increment;
            }
            else if (gamepad1.dpad_down) {
                power -= increment;
            }

            wheel1.setPower(power);
            wheel2.setPower(-power);
        }
    }
}
