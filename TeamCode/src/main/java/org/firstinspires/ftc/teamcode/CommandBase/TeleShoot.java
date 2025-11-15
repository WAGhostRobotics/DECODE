package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class TeleShoot extends SequentialCommand {
    public TeleShoot() {
        super(
                new RunCommand(()-> Bob.shooter.popUp()),
                new Wait(200),
                new RunCommand(()-> Bob.intake.rotateCW())
        );
    }
}
