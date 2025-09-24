package org.firstinspires.ftc.teamcode.RI3W;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;

@TeleOp
@Config
public class ShooterTest extends LinearOpMode {
    public static double power = 0.8;                 // Change this in dashboard at runtime
    public static double intakePower = 0;
    public static double targetVelocity = 350;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public PIDController pidController;
    public static double P = 0.01, I=0, D = 0;

    public double currentVelocity, error;

    @Override
    public void runOpMode() throws InterruptedException {
        George.init(hardwareMap);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);

        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;

            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));

            George.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            pidController.setPID(P, I, D);
            George.intake.setPower(intakePower);

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

            currentVelocity = George.wheel1.getVelocity(AngleUnit.RADIANS) * 48;  // mm
            error = targetVelocity - currentVelocity;
            power = pidController.calculate(0, error);
            power = Range.clip(power, -1, 1);


            George.intake.setPower(intakePower);
            George.wheel1.setPower(power);
            George.wheel2.setPower(power);
            George.localizer.update();


            telemetry.addData("Wheel Power: ", power);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("X: ", George.localizer.getPosX());
            telemetry.addData("Y: ", George.localizer.getPosY());
            telemetry.addData("Heading: ", George.localizer.getHeading());


            telemetry.update();

        }
    }
}
