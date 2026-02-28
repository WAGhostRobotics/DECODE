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
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV2;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

@Autonomous
public class Red15Lazer extends LinearOpMode {
    ElapsedTime timer;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    Bezier shootPath, spike1Path, spike2Path, openGatePath, openGateSpikePath, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot, rotate90, spike2intake, spike3intake;
    public static int multiplier=1;
    public static Point shootingPos = new Point(48.5, -13.6);
    public static Point farShootingPos = new Point(-125.7, 20.28);
    public static Point spike1take = new Point(48.5, 21);
    public static Point spike2 = new Point(73, -2); // **
    public static Point spike3 = new Point(96, -2.27);
    public static Point spike2take = new Point(71.2, 27.5);// **
    public static Point spike3take = new Point(96,27);

    public static Point openGate = new Point(74.3, 26.3);
    public static Point openGateSpike = new Point(67.2, 21.7);

    public static Point openGatePrepPoint = new Point(64.5, 19);

    MotionPlanner follower;
    Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 20.51, 22.50, AngleUnit.DEGREES, 90);
    int velocity = 165;
    double turretAngle = -136, hoodPos = 0.40;



    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1take = new Point(spike1take.getX(), multiplier* spike1take.getY());
        Gus.init(hardwareMap, true, false);
        follower = new MotionPlanner(Gus.drivetrain, Gus.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(90,
                new Point(startingPose.getX(DistanceUnit.INCH), startingPose.getY(DistanceUnit.INCH)),
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

        openGateSpikePath = new MergedBezier(
                90,
                new Bezier(
                        spike2take,
                        new Point(spike2.getX(), spike2.getY()+9)
                ),
                new Bezier(
                        new Point(spike2.getX(), spike2.getY()+9),
                        openGateSpike
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

        spike2intake = new MergedBezier(90,
                new Bezier(
                        shootingPos,
                        spike2
                ),
                new Bezier(
                        spike2,
                        spike2take
                )
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
                        openGateSpike,
                        new Point(spike2.getX(), spike2.getY()+9)
                ),
                new Bezier(
                        new Point(spike2.getX(), spike2.getY()+9),
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
            Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
            Gus.shooter.updateTurret();
            Gus.shooter.getTurretAngle();
        }

        waitForStart();
        while (opModeIsActive()) {
            loopRateTracker.updateLoopRate();
            scheduler.update();
            Gus.localizer.update();
            Gus.shooter.updateShooter();
            Gus.shooter.updateTurret();
            follower.update();
            Gus.shooter.getTurretAngle();
            Gus.intake.updateIntake();
            telemetry.addData("Loop Speed: ", loopRateTracker.getLoopRateHz());
            telemetry.addData("MP: ", follower.getTelemetry());
            telemetry.update();
            timer.reset();
        }
    }

    @NonNull
    private SequentialCommand getSequentialCommand() {
        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()-> Gus.localizer.setPose(startingPose)),
                new ParallelCommand(
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle)))
                ),
                new ScoreThreeArtifacts(follower, shootPath, velocity, Shooter.angleToPosition(turretAngle), hoodPos),
                new RunCommand(()-> Gus.intake.setRampFullThreshold()),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike2intake),
                        new CollectSpikesV2(follower)
                ),

                new FollowTrajectory(follower, openGateSpikePath),

                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new ParallelCommand(
                        new FollowTrajectory(follower, openGatePath),
                        new CollectBalls(follower, 1.5)
                ),
                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new ParallelCommand(
                        new FollowTrajectory(follower, openGatePath),
                        new CollectBalls(follower, 1.5)
                ),
                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

//                new ParallelCommand(
//                        new FollowTrajectory(follower, openGatePath),
//                        new CollectBalls(follower, 0.7)
//                ),
//                new ScoreThreeArtifacts(follower, spike2ToShoot, 176, Shooter.angleToPosition(-129), 0.34),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new CollectSpikesV3(follower)
                ),
                new ScoreThreeArtifacts(follower, spike1ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

//
//                new FollowTrajectory(follower, spike3Path),
//                new ParallelCommand(
//                        new FollowTrajectory(follower, spike3intake),
//                        new CollectSpikesV3(follower)
//                ),
//                new ScoreThreeArtifacts(follower, spike3ToShoot, 182, Shooter.angleToPosition(-133), 0.17),

                new ParallelCommand(
                        new FollowTrajectory(follower, rotate90),
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(0))
                )

        );
        return scheduler;
    }

}
