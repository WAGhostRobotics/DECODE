package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Gus;

@TeleOp
@Config
public class PinpointTuner extends OpMode {
    public static double xOffset;
    public static double yOffset;
    public void init() {
        Gus.init(hardwareMap, false, true);
    }

    public void loop() {
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double driveTurn = -gamepad1.right_stick_x;
        double magnitude = Math.hypot(x, y);
        double theta = Math.toDegrees(Math.atan2(y, x));
        double heading = Gus.localizer.getHeading();
        theta = normalizeDegrees(theta - heading);
        Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
        Gus.localizer.setOffsets(xOffset, yOffset);
        Gus.localizer.update();

        telemetry.addData("Loc: ", Gus.localizer.getPosX());
        telemetry.addData("Loc: ", Gus.localizer.getPosY());
        telemetry.addData("Loc: ", Gus.localizer.getHeading());
        telemetry.update();

    }
}
