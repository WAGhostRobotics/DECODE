package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class PositionFinder extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap, true, false);
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
            Bob.localizer.update();


            telemetry.addData("X: ", Bob.localizer.getPosX());
            telemetry.addData("Y: ", Bob.localizer.getPosY());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.update();
        }
    }
}
