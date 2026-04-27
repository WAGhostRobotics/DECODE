package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

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
import org.firstinspires.ftc.teamcode.CommandBase.BangBangBang;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesPedro;
import org.firstinspires.ftc.teamcode.CommandBase.FollowPedro;
import org.firstinspires.ftc.teamcode.CommandBase.GateCollectPedro;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Walt;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;

@Autonomous
public class Blue21 extends OpMode {
    File file;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    public Follower follower;
    Pose startingPose = new Pose(17.5, 113.0, Math.toRadians(180));
    Pose shootingPose = new Pose(58.686, 84.857);
    Pose spike2 = new Pose(12.686, 58.5000);
    Pose gateIntake = new Pose(12.5, 60.0);
    Pose spike1 = new Pose(18, 83.514);
    Pose spike3 = new Pose(14.457, 40.0);
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



    int velocity = 140;
    double turretAngle = 137.7, hoodPos = 0.48;



    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        Walt.init(hardwareMap, false, false);
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        preloadScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startingPose,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        spike2Path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(53.743, 58.514),
                                spike2
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        spike2ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                spike2,
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateIntakePath = new Path(
                new BezierCurve(
                        shootingPose,
                        new Pose(49, 65),
                        gateIntake
                ));
        gateIntakePath.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150));

        gateIntakePush = new Path(
                new BezierLine(
                        gateIntake,
                        new Pose(gateIntake.getX()+2.2, gateIntake.getY()-6)
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
                        new BezierLine(
                                gateIntake,
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
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
                                new Pose(52, 45.4),
                                new Pose(52, 42.8),
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
                .setLinearHeadingInterpolation(spike1ToShoot.getFinalHeadingGoal(), Math.toRadians(180))
                .build();

        follower.setStartingPose(startingPose);
        scheduler = getCommand();
        scheduler.init();

    }

    @Override
    public void init_loop() {
        Walt.shooter.setHood(hoodPos);
        Walt.shooter.updateTurret();
        Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        telemetry.addData("Is limelight chilling: ", Walt.limelight.isAlive());
        telemetry.update();

    }

    @Override
    public void loop() {
        Walt.shooter.setTargetVelocity(velocity);
        Walt.shooter.updateTurret();
        loopRateTracker.updateLoopRate();
        double heading = Math.toDegrees(follower.getHeading());
        double calcTurretAngle;
        if (follower != null && follower.getCurrentPath() != null && follower.getCurrentPathChain() != null) {
            calcTurretAngle = turretAngle -
                    (Math.toDegrees(follower.getCurrentPathChain().getFinalHeadingGoal()) - 180);
        }
        else {
            calcTurretAngle = turretAngle - heading - 180;

        }
        calcTurretAngle = normalizeDegrees(calcTurretAngle);
        Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(calcTurretAngle));
        follower.update();
        scheduler.update();
        Walt.shooter.updateShooter();
        telemetry.addData("Heading: ", heading);
        telemetry.addData("Parametric end: ", follower.atParametricEnd());
        telemetry.addData("Heading error: ", follower.getCurrentPath().getPathEndHeadingConstraint());
        telemetry.addData("T-Value: ", follower.getCurrentTValue());
        telemetry.addData("Target: ", follower.getCurrentPath().endPose().getPose());
        telemetry.addData("Pose: ", follower.poseTracker.getPose());
        telemetry.addData("X: ", follower.isBusy());
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

    public SequentialCommand getCommand() {
        return new SequentialCommand(
                new ParallelCommand(
                        new FollowPedro(follower, preloadScore),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike2Path),
                        new CollectSpikesPedro(follower)
                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike2ToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new GateCollectPedro(1.2, follower, gateIntakePath),

                new ParallelCommand(
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new GateCollectPedro(1.7, follower, gateIntakePath),

                new ParallelCommand(
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),

                new GateCollectPedro(1.7, follower, gateIntakePath),

                new ParallelCommand(
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),
                new GateCollectPedro(1.7, follower, gateIntakePath),

                new ParallelCommand(
                        new FollowPedro(follower, gateToShoot),
                        new BangBangBang(follower, 0.6, velocity)
                ),




//                new GateCollectPedro(1, follower, gateIntakePath, gateIntakePush),
//
//                new ParallelCommand(
//                        new RunCommand(()-> Gus.intake.rollerStop()),
//                        new FollowPedro(follower, gateToShoot),
//                        new BangBangBang(follower, 0.6, velocity)
//                ),

                new ParallelCommand(
                        new FollowPedro(follower, spike1Path),
                        new CollectSpikesPedro(follower)
                ),



                new ParallelCommand(
                        new FollowPedro(follower, spike1ToShoot),
                        new BangBangBang(follower, 0.7, velocity)
                ),


//
//                new ParallelCommand(
//                        new FollowPedro(follower, spike3Path),
//                        new CollectSpikesPedro(follower)
//                ),
//
//                new ParallelCommand(
//                        new FollowPedro(follower, spike3ToShoot),
//                        new BangBangBang(follower, 0.7, velocity)
//                ),

                new ParallelCommand(
                        new FollowPedro(follower, leave),
                        new RunCommand(()-> turretAngle = 0)
                )
        );
    }
}
