package org.firstinspires.ftc.teamcode.Pathing.RI3W;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.OldStuff.MotionPlannerOld;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.RI3W.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.OldStuff.FollowTrajectoryOld;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Autonomous
public class RI3W extends LinearOpMode {
    Bezier shootPath, spike1Path, spike2Path, spike1ToShoot, spike2ToShoot;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-31.7, -7.33);
    public static Point spike1 = new Point(-60.4, -10.5);
    MotionPlannerOld follower;


    @Override
    public void runOpMode() throws InterruptedException {
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        George.init(hardwareMap);
        follower = new MotionPlannerOld(George.drivetrain, George.localizer, hardwareMap);
        shootPath = new Bezier(53*multiplier,
                new Point(0, 0),
                shootingPos
        );


        spike1Path = new Bezier(90*multiplier,
                shootingPos,
                spike1
        );

        spike1ToShoot = new MergedBezier(
                new Bezier(
                        spike1,
                        new Point(spike1.getX(), 10.5)
                ),
                new Bezier(53,
                        new Point(spike1.getX(), 10.5),
                        shootingPos
                        )
        );

        SequentialCommand scheduler = new SequentialCommand(
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, shootPath),
                        new RunCommand(()->George.shooter.setTargetVelocity(180))
                ),
                new RunCommand(()->George.shooter.setIntake(0.9)),
                new Wait(3000),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike1Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.65, 0.6, 0.9 ),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike1ToShoot),
                        new RunCommand(()->George.shooter.setTargetVelocity(180))
                ),
                new RunCommand(()->George.shooter.setIntake(1))
        );

        waitForStart();
        scheduler.init();
        while (opModeIsActive()) {
            scheduler.update();
            George.localizer.update();
            George.shooter.updateShooter();
            follower.update();
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
        }
    }

}
