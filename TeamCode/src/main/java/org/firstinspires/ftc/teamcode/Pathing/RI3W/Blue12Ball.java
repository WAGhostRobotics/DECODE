package org.firstinspires.ftc.teamcode.Pathing.RI3W;

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
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikes;
import org.firstinspires.ftc.teamcode.CommandBase.CollectSpikesV3;
import org.firstinspires.ftc.teamcode.CommandBase.JankyIntakeSpike;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.ScoreThreeArtifacts;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Autonomous
public class Blue12Ball extends LinearOpMode {
    ElapsedTime timer;
    LoopRateTracker loopRateTracker = new LoopRateTracker();
    Bezier shootPath, spike1Path, spike2Path, openGatePath, spike3Path, spike3ToShoot, spike1ToShoot, spike2ToShoot, rotate90, spike2intake, spike3intake;
    public static int multiplier=1;
    public static Point shootingPos = new Point(-18.7, 44.6);
    public static Point farShootingPos = new Point(-125.7, -20.28);
    public static Point spike1 = new Point(-9.3, 47.03);
    public static Point spike2 = new Point(-12.3, 73.13);
    public static Point spike3 = new Point(-12.3, 95.4);
    public static Point spike2take = new Point(17.7, 73.13);
    public static Point spike3take = new Point(17.7,95.4);

    public static Point openGate = new Point(16, 56.5);

    MotionPlanner follower;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        shootingPos = new Point(shootingPos.getX(), multiplier* shootingPos.getY());
        spike1 = new Point(spike1.getX(), multiplier*spike1.getY());
        Bob.init(hardwareMap, true, false);
        follower = new MotionPlanner(Bob.drivetrain, Bob.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        shootPath = new Bezier(0,
                new Point(0, 0),
                shootingPos
        );

        openGatePath = new MergedBezier(
                0,
                new Bezier(
                        new Point(spike1.getX()+23, spike1.getY()),
                        new Point(openGate.getX()-10, openGate.getY())
                ),
                new Bezier(
                        new Point(openGate.getX()-10, openGate.getY()),
                        openGate
                )
        );


        spike1Path = new MergedBezier(10,
                new Bezier(
                        shootingPos,
                        new Point(spike1.getX(), spike1.getY()+1)
                ),
                new Bezier(
                        new Point(spike1.getX(), spike1.getY()+1),
                        new Point(spike1.getX()+22, spike1.getY())
                )
        );

        spike2Path = new Bezier(
                        shootingPos,
                        new Point(spike2.getX(), spike2.getY())
                );
//                new Bezier(
//                        new Point(spike2.getX(), spike2.getY()),
//                        //new Point(spike2.getX()+30, spike2.getY())
//                        spike2take
//                )
//        );

        spike2intake = new Bezier(10,
                new Point(spike2.getX(), spike2.getY()),
                spike2take
        );


        spike3Path = new Bezier(
                        shootingPos,
                        new Point(spike3.getX(), spike3.getY())
                );
//                new Bezier(
//                        new Point(spike3.getX(), spike3.getY()-7),
//                        new Point(spike3.getX()+30, spike3.getY())
//                )
//        );

        spike3intake = new Bezier(10,
                new Point(spike3.getX(), spike3.getY()),
                spike3take
        );

        spike1ToShoot = new Bezier(0,
                new Point(spike1.getX()+24, spike1.getY()),
                shootingPos
        );


        spike2ToShoot = new MergedBezier(
                new Bezier(0,
                        new Point(spike2.getX()+24, spike2.getY()),
                        new Point(spike2.getX()+12, spike2.getY())
                ),
                new Bezier(
                        new Point(spike2.getX()+12, spike2.getY()),
                        shootingPos
                )
        );

        spike3ToShoot = new Bezier(0,
                new Point(spike3.getX()+28, spike3.getY()),
                shootingPos
        );

        rotate90 = new Bezier(
                0,
                new Point(spike2.getX()+11, spike2.getY()+6)
        );

        SequentialCommand scheduler = getSequentialCommand();
        scheduler.init();
        while (opModeInInit()) {
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
            telemetry.update();
            timer.reset();
        }
    }

    @NonNull
    private SequentialCommand getSequentialCommand() {
        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()->Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(136))),
                new ScoreThreeArtifacts(follower, shootPath, 196, Shooter.angleToPosition(137), 0.30),


                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new CollectSpikesV3(follower)
                ),

//                new ParallelCommand(
//                        new RunCommand(() -> Bob.intake.rollerStop()),
//                        new FollowTrajectory(follower, openGatePath)
//                ),

                new ScoreThreeArtifacts(follower, spike1ToShoot, 196, Shooter.angleToPosition(137), 0.30),


                new FollowTrajectory(follower, spike2Path),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike2intake),
                        new CollectSpikesV3(follower)
                ),


                new ScoreThreeArtifacts(follower, spike2ToShoot, 196, Shooter.angleToPosition(137), 0.30),


                new FollowTrajectory(follower, spike3Path),
                new ParallelCommand(
                        new FollowTrajectory(follower, spike3intake),
                        new CollectSpikesV3(follower)
                ),


                new ScoreThreeArtifacts(follower, spike3ToShoot, 196, Shooter.angleToPosition(137), 0.30),

                new ParallelCommand(
                        new FollowTrajectory(follower, rotate90),
                        new RunCommand(()-> Bob.shooter.setTurretTargetPos(0))
                )

        );
        return scheduler;
    }

}
