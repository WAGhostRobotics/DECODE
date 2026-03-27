package org.firstinspires.ftc.teamcode.Pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBang;
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBangSOTM;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesPedro;
import org.firstinspires.ftc.teamcode.CommandBase.FollowPedro;
import org.firstinspires.ftc.teamcode.CommandBase.GateCollectPedro;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Red21 extends OpMode {
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    public Follower follower;
    Pose startingPose = new Pose(126.5, 113,0);
    Pose shootingPose = new Pose(85.314, 84.857);
    Pose spike2 = new Pose(129.314, 59.5000);
    Pose gateIntake = new Pose(131, 59.829);
    Pose spike1 = new Pose(127.0, 83.514);
    Pose spike3 = new Pose(129.543, 39.786);
    SequentialCommand scheduler;

    public PathChain preloadScore;
    public PathChain spike2Path;
    public PathChain spike2ToShoot;
    public Path gateIntakePath;
    public Path gateIntakePush;
    public Path gateIntakeRotate;
    public PathChain gateToShoot;
    public PathChain spike1Path;
    public PathChain spike1ToShoot;
    public PathChain spike3Path;
    public PathChain spike3ToShoot;
    public PathChain leave;



    int velocity = 155;
    int sotmVelocity = 170;
    double turretAngle = -136, hoodPos = 0.57;



    @Override
    public void init() {
        Gus.init(hardwareMap, false, false);
        follower = Constants.createFollower(hardwareMap);

        preloadScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startingPose,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        spike2Path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(90.257, 58.514),
                                spike2
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        spike2ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                spike2,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gateIntakePath = new Path(
                new BezierCurve(
                        shootingPose,
                        new Pose(95, 65),
                        gateIntake
                ));
        gateIntakePath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30));

        gateIntakePush = new Path(
                new BezierLine(
                        gateIntake,
                        new Pose(gateIntake.getX()+2.2, gateIntake.getY()-5.5)
                ));
        gateIntakePush.setConstantHeadingInterpolation(Math.toRadians(55));
        gateIntakePush.setTValueConstraint(0.73);
        gateIntakePush.setTranslationalConstraint(5);

        gateIntakeRotate = new Path(
                new BezierLine(
                        gateIntake,
                        new Pose(gateIntake.getX()-1.5, gateIntake.getY()-6)
                ));
        gateIntakeRotate.setConstantHeadingInterpolation(Math.toRadians(90));
        gateIntakeRotate.setTranslationalConstraint(5);
        gateIntakeRotate.setBrakingStart(0.1);

        gateToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateIntake,
                                new Pose(95, 61),
                                shootingPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();


        spike1Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootingPose,
                                spike1
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        spike1ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                spike1,
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
                                new Pose(95.857, 38.386),
                                new Pose(95.186, 40.786),
                                spike3

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        spike3ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                spike3,
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootingPose,
                                spike1
                        )
                )
                .setLinearHeadingInterpolation(spike3ToShoot.getFinalHeadingGoal(), 0)
                .build();

        follower.setStartingPose(startingPose);
        scheduler = getCommand();
        scheduler.init();

    }

    @Override
    public void init_loop() {
        Gus.shooter.setHood(hoodPos);
        Gus.shooter.updateTurret();
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        telemetry.addData("Is limelight chilling: ", Gus.limelight.isAlive());
        telemetry.update();

    }

    @Override
    public void loop() {
        Gus.shooter.updateTurret();
        loopRateTracker.updateLoopRate();
        double heading = Math.toDegrees(follower.getHeading());
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle - heading));
        Gus.shooter.setHood(hoodPos, true);
        follower.update();
        scheduler.update();
        Gus.shooter.updateShooter();
        if (!Gus.intake.isInitialized()) {
            Gus.intake.updateIntake();
        }
//        telemetry.addData("Heading: ", heading);
//        telemetry.addData("Parametric end: ", follower.atParametricEnd());
//        telemetry.addData("Heading error: ", follower.getCurrentPath().getPathEndHeadingConstraint());
//        telemetry.addData("T-Value: ", follower.getCurrentTValue());
//        telemetry.addData("Target: ", follower.getCurrentPath().endPose().getPose());
//        telemetry.addData("Pose: ", follower.poseTracker.getPose());
//        telemetry.addData("X: ", follower.isBusy());
        telemetry.addData("Vel: ", Gus.shooter.getTelemetry());
        telemetry.update();
    }

    public SequentialCommand getCommand() {
        return new SequentialCommand(
                new ParallelCommand(
                        new FollowPedro(follower, preloadScore),
                        new BangBangBangSOTM(follower, 0.4, sotmVelocity)
                ),

                new RunCommand(()-> Gus.intake.setRampFullThreshold()),

                new ParallelCommand(
                        new FollowPedro(follower, spike2Path),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike2ToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new GateCollectPedro(0.8, follower, gateIntakePath, gateIntakePush),

                new ParallelCommand(
                        new RunCommand(()-> Gus.intake.rollerStop()),
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new GateCollectPedro(1, follower, gateIntakePath, gateIntakePush),

                new ParallelCommand(
                        new RunCommand(()-> Gus.intake.rollerStop()),
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike1Path),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike1ToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new GateCollectPedro(0.8, follower, gateIntakePath, gateIntakePush),

                new ParallelCommand(
                        new RunCommand(()-> Gus.intake.rollerStop()),
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike3Path),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike3ToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, leave),
                        new RunCommand(()-> turretAngle = 0)
                )
        );
    }
}
