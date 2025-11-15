package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class Sort extends Command {

    ElapsedTime timer;
    double seconds;

    @Override
    public void init() {

    }

    @Override
    public void update() {
        Bob.intake.sort();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }

}
