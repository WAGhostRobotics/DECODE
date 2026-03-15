package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV2;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.GateCollect;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;
import org.firstinspires.ftc.teamcode.R;

@Autonomous
public class FarRedAuto extends OpMode {
    MotionPlanner follower;
    SequentialCommand scheduler;

    Point shootPosition = new Point(0, 0);
    Bezier shootPath, spikeToShoot, humanToShoot, tunnelToShoot, tunnelEnter, tunnelTurn, tunnelPushPath, spikePath, humanPath, leave;

    Point spike = new Point(-23.6, 1);
    Point spikeTake = new Point(-27.6, 29);
    Point human = new Point(0, 31.8);
    Point tunnel = new Point(-4, 29);
    Point tunnelPush = new Point(-24, 29);
    ElapsedTime timer = new ElapsedTime();



    int velocity = 208;
    double hoodPos = 0.12;
    double turretAngle = -106;
    @Override
    public void init() {
        Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);
        Gus.init(hardwareMap, true, false);
        follower = new MotionPlanner(Gus.drivetrain, Gus.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        leave = new Bezier(
                90,
                new Point(0, 0),
                new Point(-7, 5)
        );

        shootPath = new Bezier(90,
                new Point(startingPose.getX(DistanceUnit.INCH), startingPose.getY(DistanceUnit.INCH)),
                shootPosition
        );

        humanPath = new Bezier(90,
                shootPosition,
                human
        );

        humanToShoot = new Bezier(90,
                human,
                shootPosition
        );

        spikePath = new MergedBezier(90,
                new Bezier(
                        shootPosition,
                        spike
                ),
                new Bezier(
                        spike,
                        spikeTake
                )
        );

        spikeToShoot = new Bezier(90,
                spikeTake,
                shootPosition
        );

        tunnelEnter = new Bezier(90,
                shootPosition,
                tunnel
        );

        tunnelTurn = new Bezier(160,
                tunnel,
                tunnel
        );

        tunnelPushPath = new Bezier(160,
                tunnel,
                tunnelPush
        );

        tunnelToShoot = new Bezier(
                90,
                tunnelPush,
                shootPosition
        );




        scheduler = new SequentialCommand(
                new RunCommand(()-> Gus.localizer.setPose(startingPose)),
                new ParallelCommand(
                        new RunCommand(()-> Gus.shooter.setTargetVelocity(velocity)),
                        new RunCommand(()-> Gus.shooter.setHood(hoodPos)),
                        new RunCommand(()-> Gus.intake.openGate())
                ),

                new Wait(2500),
                new Shoot(1),

                new ParallelCommand(
                        new SequentialCommand(
                                new Wait(500),
                                new RunCommand(()-> Gus.intake.setRampFullThreshold())
                        ),
                        new FollowTrajectory(follower, humanPath),
                        new CollectSpikesV3(follower)
                ),

                new ScoreThreeArtifacts(follower, humanToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new ParallelCommand(
                        new FollowTrajectory(follower, spikePath),
                        new CollectSpikesV3(follower)
                ),
                new ScoreThreeArtifacts(follower, spikeToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),


                new GateCollect(follower, tunnelEnter, tunnelTurn, tunnelPushPath),
                new ScoreThreeArtifacts(follower, tunnelToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new GateCollect(follower, tunnelEnter, tunnelTurn, tunnelPushPath),
                new ScoreThreeArtifacts(follower, tunnelToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new GateCollect(follower, tunnelEnter, tunnelTurn, tunnelPushPath),
                new ScoreThreeArtifacts(follower, tunnelToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),


                new FollowTrajectory(follower, leave)
        );


        scheduler.init();

    }

    public void init_loop() {
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        timer.reset();
}


    public void loop() {
        if (timer.seconds() >28.5) {
            follower.startFollowingPath(leave);
        }
        else {
            scheduler.update();
        }
        Gus.localizer.update();
        Gus.shooter.updateShooter();
        Gus.shooter.setHood(hoodPos, true);
//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));

        follower.update();
        if (!Gus.intake.isInitialized()) {
            Gus.intake.updateIntake();
        }
        telemetry.addData("MP: ", follower.getTelemetry());
        telemetry.addData("Initialized: ", Gus.intake.isInitialized());
        telemetry.addData("Intake: ", Gus.intake.getTelemetry());
        telemetry.addData("Timer: ", timer.seconds());
        telemetry.update();
    }
}

