package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class ScoreThreeArtifactsSorted extends SequentialCommand {
    public ScoreThreeArtifactsSorted(MotionPlanner follower, Bezier path, double flywheelVelocity, int turretPos, double hoodPos) {
        super(
                new ParallelCommand(
                        new FollowTrajectory(follower, path),
                        new RunCommand(()->Bob.intake.sort()),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(flywheelVelocity)),
                        new RunCommand(()-> Bob.intake.rollerOut()),
                        new RunCommand(()-> Bob.shooter.setTurretTargetPos(turretPos)),
                        new RunCommand(()-> Bob.shooter.setHood(hoodPos))
                ),
                new Wait(160),
                new RunCommand(()->Bob.shooter.popUp()),
                new Wait(150),
                new RunCommand(()-> Bob.intake.rollerIn()),
                new Shoot(3),
                new RunCommand(()->Bob.shooter.stop()),
                new RunCommand(()-> Bob.intake.rollerStop()),
                new RunCommand(()->Bob.shooter.setTargetVelocity(0))
        );
    }
}
