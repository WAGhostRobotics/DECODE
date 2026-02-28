package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class OpenGate extends Command {
    MotionPlanner follower;
    public OpenGate(MotionPlanner follower) {
        this.follower = follower;
    }


    @Override
    public void init() {
    }

    @Override
    public void update() {
        if (follower.isEndOfSpline()) {
            Gus.intake.rollerStop();
            Gus.intake.openGate();
        }
    }

    @Override
    public boolean isFinished() {
        return follower.isEndOfSpline();
    }

    @Override
    public void stop() {

    }
}
