package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Autonomous
public class RI3W extends LinearOpMode {
    Bezier path;
    public static int multiplier = 1;
    MotionPlannerEdit follower;


    @Override
    public void runOpMode() throws InterruptedException {
        George.init(hardwareMap);
        follower = new MotionPlannerEdit(George.drivetrain, George.localizer, hardwareMap);
        path = new Bezier(45,
                new Point(0, 0),
                new Point(-15, 27.5)
        );

        SequentialCommand scheduler = new SequentialCommand(
                new FollowTrajectory(follower, path)
        );

        waitForStart();
        scheduler.init();
        while (opModeIsActive()) {
            scheduler.update();
            follower.update();
        }
    }

}
