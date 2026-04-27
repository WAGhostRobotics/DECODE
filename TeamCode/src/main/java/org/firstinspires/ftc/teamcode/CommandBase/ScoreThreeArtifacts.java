package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class ScoreThreeArtifacts extends SequentialCommand {
    public ScoreThreeArtifacts(MotionPlanner follower, Bezier path, double flywheelVelocity, double turretPos, double hoodPos) {
        super(
                new ParallelCommand(
                        new RunCommand(()-> Walt.intake.shootStop()),
                        new OpenGate(follower),
                        new FollowTrajectory(follower, path),
                        new RunCommand(()-> Walt.shooter.setTargetVelocity(flywheelVelocity)),
                        new RunCommand(()-> Walt.shooter.setTurretTargetPos(turretPos)),
                        new RunCommand(()-> Walt.shooter.setHood(hoodPos))
                ),
                new Shoot(0.9),
                new RunCommand(()-> Walt.shooter.setTargetVelocity(0)),
                new ParallelCommand(
                        new RunCommand(()-> Walt.intake.closeGate()),
                        new RunCommand(()-> Walt.intake.setBallIn(false))
                )
            );
    }
}
