package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class JankyIntakeSpike extends SequentialCommand {
    public JankyIntakeSpike(double movementPower, double seconds, double intakePower) {
        super(
                new ParallelCommand(
                        new RunCommand(()-> Bob.intake.rollerIn()),
                        new RunCommand(()-> Bob.shooter.setTargetVelocity(0)),
                        new RunCommand(()-> Bob.intake.setBallsEatenToZero())
                ),
                new CollectSpikes(movementPower, seconds, intakePower),
                new RunCommand(()-> Bob.shooter.setTargetVelocity(0))

        );
    }
}
