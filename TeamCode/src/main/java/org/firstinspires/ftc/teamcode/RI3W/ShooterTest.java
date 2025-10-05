package org.firstinspires.ftc.teamcode.RI3W;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;

@TeleOp
@Config
public class ShooterTest extends LinearOpMode {
    public static double power = 0.8;                 // Change this in dashboard at runtime
    public static double intakePower = 0;
    public static double targetVelocity = 0;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public static double P = 0.13, I=0.001, D = 0;
    public static double servoPos = 0;

    public double currentVelocity, error;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "s1");
        George.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;

            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));

            George.drivetrain.drive(magnitude, theta, driveTurn, 0.8);

            George.shooter.setPID(P, I, D);
            George.shooter.setIntake(intakePower);

//            if (gamepad1.dpad_up) {
//                power += increment;
//            }
//            else if (gamepad1.dpad_down) {
//                power -= increment;
//            }
            s1.setPosition(servoPos);
            if (gamepad1.right_bumper) {
                intakePower += increment;
            }
            else if (gamepad1.left_bumper) {
                intakePower -= increment;
            }

            George.shooter.setTargetVelocity(targetVelocity);
            George.shooter.updateShooter();
            George.shooter.setIntake(intakePower);

            George.localizer.update();


            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("X: ", George.localizer.getPosX());
            telemetry.addData("Y: ", George.localizer.getPosY());
            telemetry.addData("Heading: ", George.localizer.getHeading());
            telemetry.addData("ShooterCurrent: ", George.shooter.currentVelocity);
            telemetry.addData("ShooterTarget: ", George.shooter.targetVelocity);


            telemetry.update();

        }
    }
}
