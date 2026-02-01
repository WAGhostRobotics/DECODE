package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.CommandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class TeleShoot extends SequentialCommand {
    public TeleShoot() {
        super(
                new Wait(2000),
                new RunCommand(()-> Gus.intake.shoot())
        );
    }
}
