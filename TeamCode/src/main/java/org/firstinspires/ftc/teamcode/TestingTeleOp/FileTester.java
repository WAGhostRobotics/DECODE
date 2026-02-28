package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Core.Gus;

import java.io.File;

@TeleOp
public class FileTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, false, false);
        File file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        String headings = ReadWriteFile.readFile(file);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(headings);
            telemetry.update();
        }
    }
}
