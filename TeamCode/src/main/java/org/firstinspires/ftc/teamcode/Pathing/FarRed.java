package org.firstinspires.ftc.teamcode.Pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBang;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesPedro;
import org.firstinspires.ftc.teamcode.CommandBase.FollowPedro;
import org.firstinspires.ftc.teamcode.CommandBase.GateCollectPedro;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;

@Autonomous
public class FarRed extends OpMode {
    File file;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    public Follower follower;
    Pose startingPose = new Pose(88.0, 8.0,0);
    Pose shootingPose = new Pose(88.0, 11.0);
    Pose spike3 = new Pose(128.114, 35.629);
    Pose humanPlayer = new Pose(133.629, 9.029);
    Pose tunnelPoint = new Pose(130.971, 37.771);

    SequentialCommand scheduler;

    public PathChain spike3Path;
    public PathChain humanPlayerPath;
    public PathChain spikeToShoot;
    public PathChain humanPlayerToShoot;
    public PathChain tunnel;
    public PathChain tunnelToShoot;
    public PathChain leave;
    public PathChain shoot;



    int velocity = 190;
    double defaultTurretAngle = -108, hoodPos = 0.38;


    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        Gus.init(hardwareMap, false, false);
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startingPose,
                                shootingPose
                        )
                )
                .setLinearHeadingInterpolation(0, 0)
                .build();

        humanPlayerPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootingPose,
                                humanPlayer
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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
                                new Pose(102.343, 28.043),
                                spike3
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        spikeToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                spike3,
                                new Pose(97.571, 13.114),
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
                                new Pose(105.157, 8.057),
                                new Pose(113.529, 15.857),
                                new Pose(119.743, 4.514),
                                new Pose(114.429, 7.971),
                                new Pose(131.643, 25.314),
                                tunnelPoint

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        tunnelToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                tunnelPoint,
                                new Pose(97.571, 13.114),
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
        Gus.shooter.setHood(hoodPos);
        Gus.shooter.updateTurret();
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(defaultTurretAngle));
        telemetry.addData("Is limelight chilling: ", Gus.limelight.isAlive());
        telemetry.update();

    }

    @Override
    public void loop() {
        Gus.shooter.setHood(hoodPos, true);
        Gus.shooter.setTargetVelocity(velocity);
        Gus.shooter.updateTurret();
        loopRateTracker.updateLoopRate();
        double heading = Math.toDegrees(follower.getHeading());
        Gus.limelight.trackAprilTag(heading, Gus.shooter.getTurretAngle(), false, false);
        double turretAngle = Gus.limelight.getTurretAngle();
        if (turretAngle == 0) {
            turretAngle = defaultTurretAngle - heading;
        }
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        follower.update();
        scheduler.update();
        Gus.shooter.updateShooter();
        telemetry.addData("Heading: ", heading);
        telemetry.addData("Shooter: ", Gus.shooter.reachedVelocity());
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
        Gus.limelight.stop();

    }

    private SequentialCommand getCommand() {
        return new SequentialCommand(
                new RunCommand(()-> Gus.shooter.setTargetVelocity(velocity)),
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
