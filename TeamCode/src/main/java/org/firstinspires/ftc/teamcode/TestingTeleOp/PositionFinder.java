package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Core.Gus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;

@TeleOp
@Config
public class PositionFinder extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(PositionFinder.class);
    File file;
    PinpointLocalizer localizer;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double magnitude = Math.hypot(x, y);
        double theta = Math.toDegrees(Math.atan2(y, x));
        localizer.update();
//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));



        telemetry.addData("X: ", localizer.getPosX());
        telemetry.addData("Y: ", localizer.getPosY());
        telemetry.addData("Heading: ", localizer.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));
    }
}
