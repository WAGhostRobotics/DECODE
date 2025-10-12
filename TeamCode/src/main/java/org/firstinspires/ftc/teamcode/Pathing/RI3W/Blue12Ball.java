package org.firstinspires.ftc.teamcode.Pathing.RI3W;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.RI3W.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.RI3W.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Autonomous
public class Blue12Ball extends LinearOpMode {
    Bezier shootPath, spike1Path, spike2Path, openGatePath, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot, rotate90;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-40, -14.6);
    public static Point farShootingPos = new Point(-125.7, -20.28);
    public static Point spike1 = new Point(-60.5, -10.9);
    public static Point spike2 = new Point(-84.7, -10.9);
    public static Point spike3 = new Point(-107.7, -11);

    public static Point openGate = new Point(-77.6987, 13.9);

    MotionPlanner follower;


    @Override
    public void runOpMode() throws InterruptedException {
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        George.init(hardwareMap);
        follower = new MotionPlanner(George.drivetrain, George.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(49*multiplier,
                new Point(0, 0),
                shootingPos
        );

        openGatePath = new MergedBezier(
                90,
                new Bezier(
                        new Point(spike2.getX(), spike2.getY()+10),
                        new Point(openGate.getX(), spike2.getY()+5)
                ),
                new Bezier(
                        new Point(openGate.getX(), spike2.getY()+5),
                        openGate
                )
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
                new Bezier(49,
                        new Point(spike1.getX(), 10.5),
                        shootingPos
                )
        );


        spike2ToShoot = new MergedBezier(
                new Bezier(
                        openGate,
                        new Point(spike2.getX()-1, -3)
                ),
                new Bezier(49,
                        new Point(spike2.getX()-1, -5),
                        shootingPos
                )
        );

        spike3ToShoot = new MergedBezier(
                new Bezier(
                        spike3,
                        new Point(spike3.getX(), -12)
                ),
                new Bezier(
                        24,
                        new Point(spike3.getX(), -12),
                        farShootingPos
                )
        );

        rotate90 = new Bezier(
                90,
                farShootingPos
        );

        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()->George.localizer.setPose(new Pose2D(DistanceUnit.INCH, -0.618, 7.25, AngleUnit.DEGREES, 0))),

                new ScoreThreeArtifacts(follower, shootPath, 125),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike2Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),

                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.5, 0.725, 1),
                new RunCommand(()->follower.resume()),
                new FollowTrajectory(follower, openGatePath),
                new Wait(500),

                new ScoreThreeArtifacts(follower, spike2ToShoot, 125),


                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.5, 0.6, 1),
                new RunCommand(()->follower.resume()),

                new ScoreThreeArtifacts(follower, spike1ToShoot, 125),


                new ParallelCommand(
                        new FollowTrajectory(follower, spike3Path),
                        new RunCommand(()->George.shooter.setTargetVelocity(0)),
                        new RunCommand(()->George.shooter.setIntake(0.8))
                ),

                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.5, 0.65, 1),
                new RunCommand(()->follower.resume()),

                new ScoreThreeArtifacts(follower, spike3ToShoot, 170),

                new FollowTrajectory(follower, rotate90)

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
