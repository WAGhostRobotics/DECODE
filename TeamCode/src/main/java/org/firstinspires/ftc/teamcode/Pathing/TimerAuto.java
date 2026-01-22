package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Bob;

public class TimerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap, true, false);
        waitForStart();
        while (opModeIsActive()) {
            Bob.drivetrain.drive(1, 0, 0, 0.4);
        }
    }
}
