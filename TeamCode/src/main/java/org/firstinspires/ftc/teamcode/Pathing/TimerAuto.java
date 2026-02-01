package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Gus;

public class TimerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, true, false);
        waitForStart();
        while (opModeIsActive()) {
            Gus.drivetrain.drive(1, 0, 0, 0.4);
        }
    }
}
