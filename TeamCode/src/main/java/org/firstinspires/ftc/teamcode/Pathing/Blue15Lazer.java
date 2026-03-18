package org.firstinspires.ftc.teamcode.Pathing;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.MergedBezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.CollectBalls;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.GateCollect;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

import java.io.File;

@Autonomous
public class Blue15Lazer extends OpMode {
    File file;
    ElapsedTime timer;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    Bezier shootPath, spike1Path, spike2Path, openGatePath,
            openGateSpikePath, spike3Path, spike3ToShoot, spike1ToShoot,
            spike2ToShoot, rotate90, spike2intake, spike3intake, gateIntakePath, gateIntakePush;
    public static int multiplier=1;
    public static Point shootingPos = new Point(51.3, 11.7);
    public static Point spike1take = new Point(51.3, -21.5);
    public static Point spike2 = new Point(73.5, 3.27);
    public static Point spike3 = new Point(98, 2.27);
    public static Point spike2take = new Point(76.5, -28);
    public static Point spike3take = new Point(98,-28);

    public static Point openGate = new Point(75.6, -29.7);

    public static Point gateIntake = new Point(82, -31.5);
    public static Point openGatePrepPoint = new Point(68.5, -23.5);

    public static Point openGateSpike = new Point(66, -23.5);


    MotionPlanner follower;
    private Pose2D startingPos = new Pose2D(DistanceUnit.INCH, 21.69, -23.26, AngleUnit.DEGREES, -90);

    int velocity = 157;
    double turretAngle = 132, hoodPos = 0.48;
    SequentialCommand scheduler;


    @Override
    public void init() {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        timer = new ElapsedTime();
        shootingPos = new Point(shootingPos.getX(), multiplier * shootingPos.getY());
        spike1take = new Point(spike1take.getX(), multiplier * spike1take.getY());
        Gus.init(hardwareMap, true, false);
        follower = new MotionPlanner(Gus.drivetrain, Gus.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(-90,
                new Point(startingPos.getX(DistanceUnit.INCH), startingPos.getY(DistanceUnit.INCH)),
                shootingPos
        );

        openGatePath = new MergedBezier(
                -125,
                new Bezier(
                        shootingPos,
                        new Point(openGatePrepPoint.getX(), 5)
                ),
                new Bezier(
                        new Point(openGatePrepPoint.getX(), 5),
                        openGate
                )
        );

        gateIntakePath = new MergedBezier(
                new Bezier(
                        -135,
                        openGate,
                        new Point(gateIntake.getX(), openGate.getY()-6.5)
                ),
                new Bezier(
                        -135,
                        new Point(gateIntake.getX(), openGate.getY()-6.5),
                        gateIntake
                )
        );

        gateIntakePush = new Bezier(
                -180,
                gateIntake,
                new Point(gateIntake.getX()-2, gateIntake.getY()-3.5)
        );


        openGateSpikePath = new MergedBezier(
                -90,
                new Bezier(
                        spike2take,
                        new Point(spike2.getX(), spike2.getY() - 13)
                ),
                new Bezier(
                        new Point(spike2.getX(), spike2.getY() - 13),
                        openGateSpike
                )
        );


        spike1Path = new MergedBezier(-90,
                new Bezier(
                        shootingPos,
                        spike1take
                )
        );

        spike2Path = new Bezier(-90,
                shootingPos,
                new Point(spike2.getX(), spike2.getY())
        );

        spike2intake = new MergedBezier(-100,
                new Bezier(
                        shootingPos,
                        spike2
                ),
                new Bezier(
                        spike2,
                        openGateSpike
                )
        );

        spike3Path = new Bezier(-90,
                shootingPos,
                spike3
        );


        spike3intake = new MergedBezier(-90,
                new Bezier(
                        shootingPos,
                        new Point(spike3.getX() - 6, spike3.getY())
                ),
                new Bezier(
                        new Point(spike3.getX() - 6, spike3.getY()),
                        spike3take
                )
        );


        spike1ToShoot = new Bezier(-90,
                spike1take,
                shootingPos
        );


        spike2ToShoot = new MergedBezier(-90,
                new Bezier(
                        spike2take,
                        new Point(spike2.getX(), -12)
                ),
                new Bezier(
                        new Point(spike2.getX(), -12),
                        shootingPos
                )
        );

        spike3ToShoot = new Bezier(-90,
                spike3take,
                shootingPos
        );

        rotate90 = new Bezier(
                -90,
                shootingPos,
                spike1take
        );

        scheduler = getSequentialCommand();
        scheduler.init();
    }

    @Override
    public void init_loop() {
        Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle));
        Gus.shooter.updateTurret();
        Gus.shooter.getTurretAngle();
        telemetry.addData("Is limelight chilling: ", Gus.limelight.isAlive());
        telemetry.update();
    }

    @Override
    public void loop () {
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
    }

    @NonNull
    private SequentialCommand getSequentialCommand() {
        scheduler = new SequentialCommand(
                new RunCommand(()-> Gus.localizer.setPose(startingPos)),
                new ParallelCommand(
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(turretAngle)))
                ),
                new ScoreThreeArtifacts(follower, shootPath, velocity, Shooter.angleToPosition(turretAngle), hoodPos),
                new RunCommand(()-> Gus.intake.setRampFullThreshold()),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike2intake),
                        new CollectSpikesV3(follower)
                ),


                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new FollowTrajectory(follower, openGatePath),
                new Wait(300),
                new GateCollect(follower, gateIntakePath, gateIntakePush),
                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new FollowTrajectory(follower, openGatePath),
                new Wait(300),
                new GateCollect(follower, gateIntakePath, gateIntakePush),
                new ScoreThreeArtifacts(follower, spike2ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),


                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new CollectSpikesV3(follower)
                ),
                new ScoreThreeArtifacts(follower, spike1ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

//
//                new ParallelCommand(
//                        new FollowTrajectory(follower, spike3intake),
//                        new CollectSpikesV3(follower)
//                ),
//                new ScoreThreeArtifacts(follower, spike3ToShoot, velocity, Shooter.angleToPosition(turretAngle), hoodPos),

                new ParallelCommand(
                        new FollowTrajectory(follower, rotate90),
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(0))
                )

        );
        return scheduler;
    }

}
