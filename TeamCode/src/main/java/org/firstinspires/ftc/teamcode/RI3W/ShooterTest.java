package org.firstinspires.ftc.teamcode.RI3W;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class ShooterTest extends LinearOpMode {
    public static double power = 0.8;                 // Change this in dashboard at runtime
    public static double intakePower = 0;
    public static double targetVelocity = 350;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public PIDController pidController;
    public static double P, I, D = 0;

    public double currentVelocity, error;

    @Override
    public void runOpMode() throws InterruptedException {
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        DcMotorEx wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            pidController.setPID(P, I, D);
            intake.setPower(intakePower);

//            if (gamepad1.dpad_up) {
//                power += increment;
//            }
//            else if (gamepad1.dpad_down) {
//                power -= increment;
//            }

            if (gamepad1.right_bumper) {
                intakePower += increment;
            }
            else if (gamepad1.left_bumper) {
                intakePower -= increment;
            }

            currentVelocity = wheel1.getVelocity(AngleUnit.RADIANS) * 48;  // mm
            error = targetVelocity - currentVelocity;
            power = pidController.calculate(0, error);
            power = Range.clip(power, -1, 1);


            intake.setPower(intakePower);
            wheel1.setPower(power);
            wheel2.setPower(power);

            telemetry.addData("Wheel Power: ", power);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.update();

        }
    }
}
