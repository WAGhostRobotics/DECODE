package org.firstinspires.ftc.teamcode.Pathing;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV2;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

import java.io.File;

@Autonomous
public class RedPartnerLeave extends OpMode {
    File file;

    LoopRateTracker loopRateTracker = new LoopRateTracker();
    Bezier shootPath, spike1Path, spike2Path, openGatePath,
            openGateSpikePath, spike3Path, spike3ToShoot, spike1ToShoot,
            spike2ToShoot, rotate90, spike2intake, spike3intake, gateIntakePath, gateIntakePush,
            partnerLeave;
    public int multiplier=1;
    public Point shootingPos = new Point(48.5, -13.6);
    public Point farShootingPos = new Point(-125.7, 20.28);
    public Point spike1take = new Point(48.5, 21);
    public Point spike2 = new Point(72, -2); // **
    public Point spike3 = new Point(96, -2.27);
    public Point spike2take = new Point(72.8, 27.5);// **
    public Point spike3take = new Point(96,27);

    public Point openGate = new Point(74.8, 26.3);
    public Point openGateSpike = new Point(67.2, 22);
    public Point gateIntake = new Point(81, 28);

    public Point openGatePrepPoint = new Point(64.5, 19);
    public Point partner = new Point(117.6, -12);

    MotionPlanner follower;
    Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 20.51, 22.50, AngleUnit.DEGREES, 90);
    int velocity = 160;
    double turretAngle = -134, hoodPos = 0.45;
    DataLogger logger;
    SequentialCommand scheduler;




    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        shootingPos = new Point(shootingPos.getX(), multiplier * shootingPos.getY());
        spike1take = new Point(spike1take.getX(), multiplier * spike1take.getY());
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

        gateIntakePath = new MergedBezier(
                new Bezier(
                        135,
                        openGate,
                        new Point(gateIntake.getX(), openGate.getY()-6.5)
                ),
                new Bezier(
                        135,
                        new Point(gateIntake.getX(), openGate.getY()-6.5),
                        gateIntake
                )
        );

        gateIntakePush = new Bezier(
                180,
                gateIntake,
                new Point(gateIntake.getX()-2, gateIntake.getY()+3.5)
        );

        openGateSpikePath = new MergedBezier(
                90,
                new Bezier(
                        spike2take,
                        new Point(spike2.getX(), spike2.getY() + 9)
                ),
                new Bezier(
                        new Point(spike2.getX(), spike2.getY() + 9),
                        openGateSpike
                )
        );


        spike1Path = new MergedBezier(90,
                new Bezier(
                        shootingPos,
                        spike1take
                )
        );

        spike2Path = new Bezier(90,
                shootingPos,
                new Point(spike2.getX(), spike2.getY())
        );

        spike2intake = new MergedBezier(100,
                new Bezier(
                        shootingPos,
                        spike2
                ),
                new Bezier(
                        spike2,
                        openGateSpike
                )
        );



        spike3Path = new Bezier(90,
                shootingPos,
                spike3
        );

        spike3intake = new Bezier(90,
                        partner,
                        new Point(spike3.getX()+2, partner.getY()-6),
                        spike3take
        );

        partnerLeave = new MergedBezier(90,
                new Bezier(
                    shootingPos,
                    new Point(partner.getX()-3, partner.getY()-12)
                ),
                new Bezier(
                    new Point(partner.getX()-3, partner.getY()-12),
                    partner
                )
        );

        spike1ToShoot = new Bezier(90,
                spike1take,
                shootingPos
        );


        spike2ToShoot = new MergedBezier(90,
                new Bezier(
                        openGateSpike,
                        new Point(spike2.getX()-5, spike2.getY() + 12)
                ),
                new Bezier(
                        new Point(spike2.getX()-5, spike2.getY() + 12),
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
        scheduler = getSequentialCommand();
        scheduler.init();
    }

    @Override
    public void init_loop()  {
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        Gus.shooter.updateTurret();
        Gus.shooter.getTurretAngle();
        telemetry.addData("Is limelight chilling: ", Gus.limelight.isAlive());
        telemetry.update();
    }

    @Override
    public void loop()  {
        loopRateTracker.updateLoopRate();
        scheduler.update();
        Gus.localizer.update();
        Gus.shooter.updateShooter();
//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));

        follower.update();
        if (!Gus.intake.isInitialized()) {
            Gus.intake.updateIntake();
        }
        telemetry.addData("Loop Speed: ", loopRateTracker.getLoopRateHz());
        telemetry.addData("MP: ", follower.getTelemetry());
        telemetry.update();
    }

    @Override
    public void stop() {
        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));
//        Gus.limelight.stop();
    }
    @NonNull
    private SequentialCommand getSequentialCommand() {
        scheduler = new SequentialCommand(
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


                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

//                new FollowTrajectory(follower, openGatePath),
//                new Wait(300),
//                new GateCollect(follower, gateIntakePath, gateIntakePush),
//                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

//                new FollowTrajectory(follower, openGatePath),
//                new Wait(300),
//                new GateCollect(follower, gateIntakePath, gateIntakePush),
//                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),


//                new ParallelCommand(
//                        new FollowTrajectory(follower, openGatePath),
//                        new CollectBalls(follower, 1.5)
//                ),
//                new ScoreThreeArtifacts(follower, spike2ToShoot, 165, Shooter.angleToPosition(-136), 0.40),

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
                new FollowTrajectory(follower, partnerLeave),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike3intake),
                        new CollectSpikesV3(follower)
                ),
                new ScoreThreeArtifacts(follower, spike3ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new ParallelCommand(
                        new FollowTrajectory(follower, rotate90),
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(0)))
                )

        );
        return scheduler;
    }
}
