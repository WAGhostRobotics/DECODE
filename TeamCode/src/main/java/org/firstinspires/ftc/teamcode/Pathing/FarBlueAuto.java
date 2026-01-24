package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Autonomous
public class FarBlueAuto extends LinearOpMode {
    MotionPlanner follower;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap, true, false);
        follower = new MotionPlanner(Bob.drivetrain, Bob.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        Bezier leave = new Bezier(
                -90,
                new Point(0, 0),
                new Point(-7, -18)
        );

        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()-> Bob.shooter.setHood(0)),
                new RunCommand(()-> Bob.shooter.setTurretTargetPos(-4200)),
                new RunCommand(()-> Bob.shooter.setTargetVelocity(216)),
                new RunCommand(()-> Bob.intake.openGate()),
                new Wait(1000),
                new Shoot(10.0),
                new FollowTrajectory(follower, leave)
        );

        while (opModeInInit()) {
            Bob.shooter.getTurretAngle();
            Bob.shooter.setTurretTargetPos(-4200);
            Bob.shooter.updateTurret();
        }

        waitForStart();
        scheduler.init();

        while (opModeIsActive()) {
            Bob.localizer.update();
            follower.update();
            Bob.shooter.updateShooter();
            Bob.shooter.updateTurret();
            scheduler.update();
            Bob.shooter.getTurretAngle();
        }
    }
}
