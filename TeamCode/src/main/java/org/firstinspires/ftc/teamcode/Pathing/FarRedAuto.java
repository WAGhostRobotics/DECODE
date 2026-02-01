package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.Shoot;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Gus;

@Autonomous
public class FarRedAuto extends LinearOpMode {
    MotionPlanner follower;
    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, true, false);
        follower = new MotionPlanner(Gus.drivetrain, Gus.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        Bezier leave = new Bezier(
                90,
                new Point(0, 0),
                new Point(-7, 18)
        );

        SequentialCommand scheduler = new SequentialCommand(
                new RunCommand(()-> Gus.shooter.setHood(0)),
                new RunCommand(()-> Gus.shooter.setTurretTargetPos(4200)),
                new RunCommand(()-> Gus.shooter.setTargetVelocity(216)),
                new RunCommand(()-> Gus.intake.openGate()),
                new Wait(1000),
                new Shoot(10.0),
                new FollowTrajectory(follower, leave)
        );

        while (opModeInInit()) {
            Gus.shooter.getTurretAngle();
            Gus.shooter.setTurretTargetPos(4200);
            Gus.shooter.updateTurret();
        }

        waitForStart();
        scheduler.init();

        while (opModeIsActive()) {
            Gus.localizer.update();
            follower.update();
            Gus.shooter.updateShooter();
            Gus.shooter.updateTurret();
            scheduler.update();
            Gus.shooter.getTurretAngle();
        }
    }
}
