package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Core.Gus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;

@TeleOp
@Config
public class PositionFinder extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(PositionFinder.class);
    File file;

    @Override
    public void init() {
        Gus.init(hardwareMap, true, false);
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double magnitude = Math.hypot(x, y);
        double theta = Math.toDegrees(Math.atan2(y, x));
        Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
        Gus.localizer.update();
        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));



        telemetry.addData("X: ", Gus.localizer.getPosX());
        telemetry.addData("Y: ", Gus.localizer.getPosY());
        telemetry.addData("Heading: ", Gus.localizer.getHeading());
        telemetry.addData("X Velocity: ", Gus.localizer.getXVelocity());
        telemetry.addData("Y Velocity: ", Gus.localizer.getYVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));
    }
}
