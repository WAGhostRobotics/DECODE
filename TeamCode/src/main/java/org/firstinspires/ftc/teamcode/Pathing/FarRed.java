package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBang;
import org.firstinspires.ftc.teamcode.CommandBase.CollectHP;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesPedro;
import org.firstinspires.ftc.teamcode.CommandBase.FollowPedro;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Walt;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous
public class FarRed extends OpMode {
    File file;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    public Follower follower;
    Pose startingPose = new Pose(88.0, 8.0,0);
    Pose shootingPose = new Pose(88.0, 17.0);
    Pose spike3 = new Pose(128.114, 35.800);
    Pose humanPlayer = new Pose(130.0, 8.6);
    Pose tunnelPoint = new Pose(130.971, 32.0);
    Pose newTunnelPoint = new Pose(130.057, 29.542);

    SequentialCommand scheduler;

    public PathChain spike3Path;
    public PathChain humanPlayerPath;
    public PathChain spikeToShoot;
    public PathChain humanPlayerToShoot;
    public PathChain tunnel;
    public PathChain newTunnel;
    public PathChain tunnelToShoot;
    public PathChain newTunnelToShoot;
    public PathChain leave;
    public PathChain shoot;



    int velocity = 179;
    double defaultTurretAngle = -110.0, hoodPos = 0.38;


    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        Walt.init(hardwareMap, false, false);
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
                        new BezierCurve(
                                shootingPose,
                                new Pose(134.229, 19.800),
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
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        spike3Path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startingPose,
                                new Pose(93.857, 30.643),
                                spike3
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        spikeToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                spike3,
                                new Pose(97.571, 13.114),
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        newTunnel = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(108.5, 29.31),
                                new Pose(130.6, 36.77),
                                newTunnelPoint

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        tunnel = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(106.457, 7.714),
                                new Pose(113.529, 15.857),
                                new Pose(134.914, 0.400),
                                new Pose(128.315, 14.400),
                                new Pose(131.129, 26.085),
                                tunnelPoint

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        tunnelToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                tunnelPoint,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        newTunnelToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                newTunnelPoint,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
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

    }

    @Override
    public void loop() {
        Walt.shooter.setHood(hoodPos, true);
        Walt.shooter.setTargetVelocity(velocity);
        Walt.shooter.updateTurret();
        loopRateTracker.updateLoopRate();
        double heading = Math.toDegrees(follower.getHeading());
//        double turretAngle = Walt.limelight.getTurretAngle();
        double turretAngle = 0;
        if (follower != null && follower.getCurrentPath() != null && follower.getCurrentPathChain() != null) {
            turretAngle = defaultTurretAngle -
                    (Math.toDegrees(follower.getCurrentPathChain().getFinalHeadingGoal())) ;
        }
        else {
            turretAngle = defaultTurretAngle - heading;

        }

        turretAngle = normalizeDegrees(turretAngle);
        turretAngle = Range.clip(turretAngle, -135, -90);
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
        double x = 144 - follower.getPose().getX();
        double y = 144 - follower.getPose().getY();
        String pose = x + "," + y + "," + (Math.toDegrees(follower.getHeading())+180);

        ReadWriteFile.writeFile(
                file,
                pose
        );
        Walt.limelight.stop();
    }

    private SequentialCommand getCommand() {
        return new SequentialCommand(
                new RunCommand(()-> Walt.shooter.setTargetVelocity(velocity)),
                new Shoot(0.5),


                new ParallelCommand(
                        new FollowPedro(follower, spike3Path),
                        new CollectSpikesPedro(follower)
                ),
                new RunCommand(() -> defaultTurretAngle = -113.5),
                new RunCommand(() -> velocity = 176),


                new ParallelCommand(
                        new FollowPedro(follower, spikeToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),
                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerPath),
                        new CollectHP(follower, 0.5)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),

                new RunCommand(() -> defaultTurretAngle = -114.5),


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

//                new ParallelCommand(
//                        new FollowPedro(follower, newTunnel),
//                        new CollectSpikesPedro(follower)
//                ),
//
//
//                new ParallelCommand(
//                        new FollowPedro(follower, newTunnelToShoot),
//                        new BangBangBang(follower, 0.7, velocity)
//                ),

                new ParallelCommand(
                        new FollowPedro(follower, tunnel),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, tunnelToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),
                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerPath),
                        new CollectSpikesPedro(follower)
                ),


                new ParallelCommand(
                        new FollowPedro(follower, humanPlayerToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                )

        );
    }
}
