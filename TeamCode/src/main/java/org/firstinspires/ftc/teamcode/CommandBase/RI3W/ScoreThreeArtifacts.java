package org.firstinspires.ftc.teamcode.CommandBase.RI3W;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Path;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.RI3W.George;

public class ScoreThreeArtifacts extends SequentialCommand {
    public ScoreThreeArtifacts(MotionPlanner follower, Bezier path, double flywheelVelocity) {
        super(
                new ParallelCommand(
                        new FollowTrajectory(follower, path),
                        new RunCommand(()-> George.shooter.setTargetVelocity(flywheelVelocity))
                ),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(1000),
                new RunCommand(()->George.shooter.setIntake(-0.35)),
                new Wait(500),
                new RunCommand(()->George.shooter.setIntake(1)),
                new Wait(800)
        );
    }
}
