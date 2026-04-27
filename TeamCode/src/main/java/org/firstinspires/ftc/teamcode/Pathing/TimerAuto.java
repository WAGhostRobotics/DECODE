package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Walt;

public class TimerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Walt.init(hardwareMap, true, false);
        waitForStart();
        while (opModeIsActive()) {
            Walt.drivetrain.drive(1, 0, 0, 0.4);
        }
    }
}
