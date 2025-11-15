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
import org.firstinspires.ftc.teamcode.CommandBase.DetectMotif;
import org.firstinspires.ftc.teamcode.CommandBase.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifactsSorted;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Autonomous
public class Red9Ball extends LinearOpMode {
    Bezier detectPath, shootPath, spike1Path, spike2Path, openGatePath, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot, rotate90;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-47, 20.6);

    public Point detectPoint = new Point(-36, 9.6);
    public static Point spike1 = new Point(-51.5, 7.8);
    public static Point spike2 = new Point(-74, 10);
    public static Point spike3 = new Point(-97.7, 3);

    public static Point openGate = new Point(-80, 13.9);

    MotionPlanner follower;


    @Override
    public void runOpMode() throws InterruptedException {
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        Bob.init(hardwareMap);
        Bob.limelight.switchToMotifPipeline();
        follower = new MotionPlanner(Bob.drivetrain, Bob.localizer, hardwareMap);
        follower.setMovementPower(0.96);

        detectPath = new Bezier(
                0,
                new Point(0, 0),
                detectPoint
        );

        shootPath = new Bezier(-90*multiplier,
                detectPoint,
                shootingPos
        );


        spike1Path = new Bezier(-90*multiplier,
                shootingPos,
                new Point(spike1.getX(), spike1.getY()+12),
                spike1
        );

        spike2Path = new Bezier(-90,
                shootingPos,
                new Point(spike2.getX(), spike2.getY()+12),
                spike2
        );

        spike3Path = new Bezier(-90,
                shootingPos,
                new Point(spike3.getX()+10, spike3.getY()+4),
                spike3
        );

        spike1ToShoot = new MergedBezier(
                new Bezier(
                        spike1,
                        new Point(spike1.getX(), 3)
                ),
                new Bezier(-90,
                        new Point(spike1.getX(), 3),
                        shootingPos
                )
        );


        spike2ToShoot = new MergedBezier(
                -90,
                new Bezier(
                        -90,
                        spike2,
                        new Point(spike2.getX()-1, 3)
                ),
                new Bezier(
                        -90,
                        new Point(spike2.getX()-1, 1),
                        shootingPos
                )
        );

        spike3ToShoot = new MergedBezier(
                -90,
                new Bezier(
                        spike3,
                        new Point(spike3.getX(), 3)
                ),
                new Bezier(-90,
                        new Point(spike3.getX(), 3),
                        shootingPos
                )
        );

        rotate90 = new Bezier(
                -90,
                new Point(spike2.getX()+11, spike2.getY()-6)
        );

        SequentialCommand scheduler = new SequentialCommand(
                new ParallelCommand(
                        new RunCommand(()-> Bob.localizer.setPose(new Pose2D(DistanceUnit.INCH, -0.618, -7.25, AngleUnit.DEGREES, 0))),
                        new RunCommand(()-> Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(41)))
                ),
                new FollowTrajectory(follower, detectPath),
                new RunCommand(()-> Bob.shooter.setTargetVelocity(200)),
                new DetectMotif(0.5),
                new ScoreThreeArtifactsSorted(follower, shootPath, 200, Shooter.angleToPosition(45), 0.385),


                new ParallelCommand(
                        new RunCommand(()->Bob.intake.holdAtZero()),
                        new FollowTrajectory(follower, spike1Path),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(0)),
                        new RunCommand(()-> Bob.shooter.setIntake(1))

                ),
                new Wait(10),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.18 , 3, 1),
                new RunCommand(()->follower.resume()),
//                new FollowTrajectory(follower, openGatePath),
//                new Wait(500),
                new ScoreThreeArtifactsSorted(follower, shootPath, 200, Shooter.angleToPosition(45), 0.385),


                new ParallelCommand(
                        new FollowTrajectory(follower, spike2Path),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(0)),
                        new RunCommand(()-> Bob.shooter.setIntake(1)),
                        new RunCommand(()->Bob.intake.holdAtZero())

                ),
//                new Wait(100000),

                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.18 , 3, 1),
                new RunCommand(()->follower.resume()),

                new ScoreThreeArtifactsSorted(follower, shootPath, 200, Shooter.angleToPosition(45), 0.385)
        );
        scheduler.init();
        while (opModeInInit()) {
            Bob.shooter.updateTurret();
            Bob.shooter.getTurretAngle();
        }
        waitForStart();
        while (opModeIsActive()) {
            scheduler.update();
            Bob.localizer.update();
            Bob.shooter.updateShooter();
            Bob.shooter.updateTurret();
            Bob.intake.updateSpindexer();
            follower.update();
//            telemetry.addData("", follower.getTelemetry());
            Bob.shooter.getTurretAngle();
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.update();
        }
    }

}
