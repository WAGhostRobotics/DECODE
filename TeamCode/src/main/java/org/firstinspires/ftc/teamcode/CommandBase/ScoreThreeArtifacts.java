package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class ScoreThreeArtifacts extends SequentialCommand {
    public ScoreThreeArtifacts(MotionPlanner follower, Bezier path, double flywheelVelocity, double turretPos, double hoodPos) {
        super(
                new ParallelCommand(
                        new RunCommand(()-> Gus.intake.shootStop()),
                        new OpenGate(follower),
                        new FollowTrajectory(follower, path),
                        new RunCommand(()-> Gus.shooter.setTargetVelocity(flywheelVelocity)),
                        new RunCommand(()-> Gus.shooter.setTurretTargetPos(turretPos)),
                        new RunCommand(()-> Gus.shooter.setHood(hoodPos))
                ),
                new Wait(100),  // Remove later
                new Shoot(1.2),
                new RunCommand(()-> follower.forceComplete()),
                new RunCommand(()-> Gus.shooter.setTargetVelocity(0)),
                new ParallelCommand(
                        new RunCommand(()-> Gus.intake.closeGate()),
                        new RunCommand(()-> Gus.intake.setBallIn(false))
                )
            );
    }
}
