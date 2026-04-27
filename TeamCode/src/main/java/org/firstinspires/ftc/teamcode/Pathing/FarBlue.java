package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBang;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesPedro;
import org.firstinspires.ftc.teamcode.CommandBase.FollowPedro;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Walt;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;

@Autonomous
public class FarBlue extends OpMode {
    File file;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    public Follower follower;
    Pose startingPose = new Pose(56.0, 8.0, Math.toRadians(180));
    Pose shootingPose = new Pose(56.0, 11.0);
    Pose spike3 = new Pose(15.886, 35.629);
    Pose humanPlayer = new Pose(13.0, 8.6);
    Pose tunnelPoint = new Pose(11.0, 37.771);

    SequentialCommand scheduler;
    ElapsedTime timer;

    public PathChain spike3Path;
    public PathChain humanPlayerPath;
    public PathChain spikeToShoot;
    public PathChain humanPlayerToShoot;
    public PathChain tunnel;
    public PathChain tunnelToShoot;
    public PathChain leave;
    public PathChain shoot;



    int velocity = 186;
    double defaultTurretAngle = 112.7, hoodPos = 0.38;


    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        Walt.init(hardwareMap, false, false);
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        timer = new ElapsedTime();

        shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startingPose,
                                shootingPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        humanPlayerPath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(9.771, 19.800),
                                humanPlayer
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        humanPlayerToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                humanPlayer,
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        spike3Path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(50.143, 30.643),
                                spike3
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        spikeToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                spike3,
                                new Pose(46.429, 13.114),
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        tunnel = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(37.543, 7.714),
                                new Pose(30.471, 15.857),
                                new Pose(9.086, 0.400),
                                new Pose(15.685, 14.400),
                                new Pose(12.971, 26.085),
                                tunnelPoint

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        tunnelToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                tunnelPoint,
                                new Pose(46.429, 13.114),
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        follower.setStartingPose(startingPose);
        scheduler = getCommand();
        scheduler.init();
    }


    @Override
    public void init_loop() {
        Walt.shooter.setHood(hoodPos);
        Walt.shooter.updateTurret();
        Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(defaultTurretAngle));
        telemetry.addData("Is limelight chilling: ", Walt.limelight.isAlive());
        telemetry.update();
        timer.reset();

    }

    @Override
    public void loop() {
        Walt.shooter.setHood(hoodPos, true);
        Walt.shooter.setTargetVelocity(velocity);
        Walt.shooter.updateTurret();
        loopRateTracker.updateLoopRate();
        double turretAngle = 0;
        double heading = Math.toDegrees(follower.getHeading());

//        double turretAngle = Walt.limelight.getTurretAngle();
        if (follower != null && follower.getCurrentPath() != null && follower.getCurrentPathChain() != null) {
            turretAngle = defaultTurretAngle -
                    (Math.toDegrees(follower.getCurrentPathChain().getFinalHeadingGoal()) - 180) ;
        }
        else {
            turretAngle = defaultTurretAngle - heading - 180;

        }

        turretAngle = normalizeDegrees(turretAngle);
        turretAngle = Range.clip(turretAngle, 90, 135);
        Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        follower.update();
        scheduler.update();
        Walt.shooter.updateShooter();
        telemetry.addData("Heading: ", heading);
        telemetry.addData("Shooter: ", Walt.shooter.reachedVelocity());
        telemetry.addData("Turret Angle: ", turretAngle);

        if (follower != null && follower.getCurrentPath() != null) {
            telemetry.addData("Parametric end: ", follower.atParametricEnd());
            telemetry.addData("T-Value: ", follower.getCurrentTValue());
//            telemetry.addData("Target: ", follower.getCurrentPath().endPose().getPose());
//            telemetry.addData("Pose: ", follower.poseTracker.getPose());
            telemetry.addData("X: ", follower.isBusy());
            telemetry.addData("Finished: ", PedroUtil.isFinished(follower));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        ReadWriteFile.writeFile(
                file,
                Double.toString(
                        Math.toDegrees(follower.getHeading())
                )
        );
        Walt.limelight.stop();

    }

    private SequentialCommand getCommand() {
        return new SequentialCommand(
                new RunCommand(()-> Walt.shooter.setTargetVelocity(velocity)),
                new Shoot(0.5),
                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerPath),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike3Path),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spikeToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),
                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                )


        );
    }
}
