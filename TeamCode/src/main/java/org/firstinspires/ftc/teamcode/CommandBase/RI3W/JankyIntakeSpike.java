package org.firstinspires.ftc.teamcode.CommandBase.RI3W;

import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.RI3W.George;

public class JankyIntakeSpike extends SequentialCommand {
    public JankyIntakeSpike(double movementPower, double seconds, double intakePower) {
        super(
                new RunCommand(()->George.shooter.setTargetVelocity(0)),
                new CollectSpikes(movementPower, seconds, intakePower),
//                new RunCommand(()-> George.shooter.setIntake(-0.3)),
//                new Wait(30),
                new RunCommand(()-> George.shooter.setIntake(0)),
//                new Wait(40),
                new RunCommand(()->George.shooter.setTargetVelocity(0))

        );
    }
}
