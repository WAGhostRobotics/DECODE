package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class ScoreThreeArtifacts extends SequentialCommand {
    public ScoreThreeArtifacts(MotionPlanner follower, Bezier path, double flywheelVelocity, int turretPos, double hoodPos) {
        super(
                new RunCommand(()-> Bob.intake.rollerStop()),
                new ParallelCommand(
                        new FollowTrajectory(follower, path),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(flywheelVelocity)),
                        new RunCommand(()-> Bob.shooter.setTurretTargetPos(turretPos)),
                        new RunCommand(()-> Bob.shooter.setHood(hoodPos))
                ),
                new Shoot(5),
                new RunCommand(()->Bob.shooter.setTargetVelocity(0))
        );
    }
}
