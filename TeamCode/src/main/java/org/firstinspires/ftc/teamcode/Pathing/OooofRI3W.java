package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Autonomous
public class OooofRI3W extends LinearOpMode {
    Bezier shootPath, spike1Path, spike2Path, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-40, -14.6);
    public static Point farShootingPos = new Point(-125.7, -20.28);
    public static Point spike1 = new Point(-60.5, -10.8);
    public static Point spike2 = new Point(-84.4, -10.8);
    public static Point spike3 = new Point(-107.7, -10.95);

    MotionPlanner follower;


    @Override
    public void runOpMode() throws InterruptedException {
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        George.init(hardwareMap);
        follower = new MotionPlanner(George.drivetrain, George.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(48*multiplier,
                new Point(0, 0),
                shootingPos
        );


        spike1Path = new Bezier(90*multiplier,
                shootingPos,
                spike1
        );

        spike2Path = new Bezier(90,
                shootingPos,
                spike2
        );

        spike3Path = new Bezier(90,
                shootingPos,
                spike3
        );

        spike1ToShoot = new MergedBezier(
                new Bezier(
                        spike1,
                        new Point(spike1.getX(), 10.5)
                ),
                new Bezier(48,
                        new Point(spike1.getX(), 10.5),
                        shootingPos
                )
        );


        spike2ToShoot = new MergedBezier(
                new Bezier(
                        spike2,
                        new Point(spike2.getX()-1, -10)
                ),
                new Bezier(48,
                        new Point(spike2.getX()-1, -10),
                        shootingPos
                )
        );

        spike3ToShoot = new MergedBezier(
                new Bezier(
                        spike3,
                        new Point(spike3.getX(), -12)
                ),
                new Bezier(
                        23.4,
                        new Point(spike3.getX(), -12),
                        farShootingPos
                )
        );

        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()->George.localizer.setPose(new Pose2D(DistanceUnit.INCH, -0.618, 7.25, AngleUnit.DEGREES, 0))),
                new ParallelCommand(
                        new FollowTrajectory(follower, shootPath),
                        new RunCommand(()->George.shooter.setTargetVelocity(122))
                ),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1200),
                new RunCommand(()->George.shooter.setIntake(-0.2)),
                new Wait(500),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1000),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.6, 0.64, 0.85),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike1ToShoot),
                        new RunCommand(()->George.shooter.setTargetVelocity(122))
                ),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1200),
                new RunCommand(()->George.shooter.setIntake(-0.2)),
                new Wait(500),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1000),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike2Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.6, 0.645, 0.85),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike2ToShoot),
                        new RunCommand(()->George.shooter.setTargetVelocity(122))
                ),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1200),
                new RunCommand(()->George.shooter.setIntake(-0.2)),
                new Wait(500),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1000),



                new ParallelCommand(
                        new FollowTrajectory(follower, spike3Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.65, 0.65, 0.85),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike3ToShoot),
                        new RunCommand(()->George.shooter.setTargetVelocity(172))
                ),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1200),
                new RunCommand(()->George.shooter.setIntake(-0.2)),
                new Wait(500),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1000)

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
