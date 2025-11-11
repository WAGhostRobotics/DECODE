package org.firstinspires.ftc.teamcode.Pathing.RI3W;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.OldStuff.MotionPlannerOld;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.OldStuff.FollowTrajectoryOld;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Autonomous
public class OooofRI3WRed extends LinearOpMode {
    Bezier shootPath, spike1Path, spike2Path, spike1ToShoot, spike2ToShoot;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-19.3, 5.1);
    public static Point spike1 = new Point(-41.5, -6);
    public static Point spike2 = new Point(-65.5, -5.75);
    MotionPlannerOld follower;


    @Override
    public void runOpMode() throws InterruptedException {
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        Bob.init(hardwareMap);
        follower = new MotionPlannerOld(Bob.drivetrain, Bob.localizer, hardwareMap);
        shootPath = new Bezier(-55.5,
                new Point(0, 0),
                shootingPos
        );


        spike1Path = new Bezier(-90,
                shootingPos,
                new Point(-38, 6),
                spike1
        );
        spike2Path = new Bezier(-90,
                shootingPos,
                new Point(-58, 6),
                new Point(-65, 5),
                spike2
        );

        spike1ToShoot = new Bezier(-55.5,
                spike1,
                shootingPos
        );

        spike2ToShoot = new Bezier(-55.5,
                spike2,
                shootingPos
        );

        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()-> Bob.localizer.setPose(new Pose2D(DistanceUnit.INCH, -0.618, -7.25, AngleUnit.DEGREES, 0))),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, shootPath),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(186))
                ),
                new Wait(500),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(2000),
                new RunCommand(()-> Bob.shooter.setIntake(-0.2)),
                new Wait(700),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(1000),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike1Path),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(0)),
                        new RunCommand(()-> Bob.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.65, 0.6, 0.75),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike1ToShoot),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(186))
                ),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(2000),
                new RunCommand(()-> Bob.shooter.setIntake(-0.2)),
                new Wait(700),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(1000),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike2Path),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(0)),
                        new RunCommand(()-> Bob.shooter.setIntake(0.8))
                ),
                new RunCommand(()->follower.pause()),
                new JankyIntakeSpike(0.65, 0.6, 0.75),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new FollowTrajectoryOld(follower, spike2ToShoot),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(186))
                ),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(2000),
                new RunCommand(()-> Bob.shooter.setIntake(-0.2)),
                new Wait(700),
                new RunCommand(()-> Bob.shooter.setIntake(1)),
                new Wait(1000)
        );

        waitForStart();
        scheduler.init();
        while (opModeIsActive()) {
            scheduler.update();
            Bob.localizer.update();
            Bob.shooter.updateShooter();
            follower.update();
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
        }
    }

}
