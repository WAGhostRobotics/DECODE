package org.firstinspires.ftc.teamcode.Pathing;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.CollectBalls;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Autonomous
public class RedGateAuto extends LinearOpMode {
    ElapsedTime timer;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    Bezier shootPath, spike1Path, spike2Path, openGatePath, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot, rotate90, spike2intake, spike3intake;
    public static int multiplier=1;
    public static Point shootingPos = new Point(48.5, -13.6);
    public static Point farShootingPos = new Point(-125.7, 20.28);
    public static Point spike1take = new Point(48.5, 21);
    public static Point spike2 = new Point(73.5, -2);
    public static Point spike3 = new Point(96, -2.27);
    public static Point spike2take = new Point(72.2, 27);
    public static Point spike3take = new Point(96,27);

    public static Point openGate = new Point(73.9, 27.2);
    public static Point openGatePrepPoint = new Point(66.5, 21);

    MotionPlanner follower;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1take = new Point(spike1take.getX(), multiplier* spike1take.getY());
        Bob.init(hardwareMap, true, false);
        follower = new MotionPlanner(Bob.drivetrain, Bob.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(90,
                new Point(0, 0),
                shootingPos
        );

        openGatePath = new MergedBezier(
                125,
                new Bezier(
                        shootingPos,
                        new Point(openGatePrepPoint.getX(), -5)
                ),
                new Bezier(
                        new Point(openGatePrepPoint.getX(), -5),
                        openGate
                )
        );



        spike1Path = new MergedBezier(90,
                new Bezier(
                        shootingPos,
                        spike1take
                )
        );

        spike2Path = new Bezier( 90,
                shootingPos,
                new Point(spike2.getX(), spike2.getY())
        );

        spike2intake = new Bezier(90,
                spike2,
                spike2take
        );


        spike3Path = new Bezier(90,
                shootingPos,
                spike3
        );

        spike3intake = new Bezier(90,
                new Point(spike3.getX(), spike3.getY()),
                spike3take
        );

        spike1ToShoot = new Bezier(90,
                spike1take,
                shootingPos
        );


        spike2ToShoot = new MergedBezier(90,
                new Bezier(
                        spike2take,
                        new Point(spike2.getX(), 16.5)
                ),
                new Bezier(
                        new Point(spike2.getX(), 16.5),
                        shootingPos
                )
        );

        spike3ToShoot = new Bezier(90,
                spike3take,
                shootingPos
        );

        rotate90 = new Bezier(
                90,
                shootingPos,
                spike1take
        );

        SequentialCommand scheduler = getSequentialCommand();
        scheduler.init();
        while (opModeInInit()) {
            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(-133));
            Bob.shooter.updateTurret();
            Bob.shooter.getTurretAngle();
        }

        waitForStart();
        while (opModeIsActive()) {
            loopRateTracker.updateLoopRate();
            scheduler.update();
            Bob.localizer.update();
            Bob.shooter.updateShooter();
            Bob.shooter.updateTurret();
            follower.update();
            Bob.shooter.getTurretAngle();
            telemetry.addData("Loop Speed: ", loopRateTracker.getLoopRateHz());
            telemetry.addData("MP: ", follower.getTelemetry());
            telemetry.update();
            timer.reset();
        }
    }

    @NonNull
    private SequentialCommand getSequentialCommand() {
        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()-> Bob.localizer.setPose(new Pose2D(DistanceUnit.INCH, 1.7, 13.57, AngleUnit.DEGREES, 38.5))),
                new ParallelCommand(
                        new RunCommand(()->Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(-133)))
                ),
                new ScoreThreeArtifacts(follower, shootPath, 182, Shooter.angleToPosition(-133), 0.17),

                new FollowTrajectory(follower, spike2Path),
                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(0.9)),
                        new FollowTrajectory(follower, spike2intake),
                        new CollectSpikesV3(follower)
                ),
                new RunCommand(()-> follower.setMovementPower(0.9)),
                new ScoreThreeArtifacts(follower, spike2ToShoot, 182, Shooter.angleToPosition(-133), 0.17),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(0.9)),
                        new FollowTrajectory(follower, openGatePath),
                        new CollectBalls(follower, 1)
                ),
                new RunCommand(()-> follower.setMovementPower(0.9)),
                new ScoreThreeArtifacts(follower, spike2ToShoot, 182, Shooter.angleToPosition(-133), 0.17),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(0.9)),
                        new FollowTrajectory(follower, spike1Path),
                        new CollectSpikesV3(follower)
                ),
                new RunCommand(()-> follower.setMovementPower(0.9)),
                new ScoreThreeArtifacts(follower, spike1ToShoot, 182, Shooter.angleToPosition(-133), 0.17),


                new FollowTrajectory(follower, spike3Path),
                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(0.9)),
                        new FollowTrajectory(follower, spike3intake),
                        new CollectSpikesV3(follower)
                ),
                new RunCommand(()-> follower.setMovementPower(0.9)),
                new ScoreThreeArtifacts(follower, spike3ToShoot, 182, Shooter.angleToPosition(-133), 0.17),

                new ParallelCommand(
                        new FollowTrajectory(follower, rotate90),
                        new RunCommand(()-> Bob.shooter.setTurretTargetPos(0))
                )

        );
        return scheduler;
    }

}
