package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class CollectSpikesV3 extends Command {
    MotionPlanner follower;
    public CollectSpikesV3(MotionPlanner follower) {
        this.follower = follower;
    }


    @Override
    public void init() {
        Gus.intake.closeGate();
        Gus.intake.setBallIn(false);
        Gus.intake.rollerIn();
    }

    @Override
    public void update() {
        Gus.intake.updateIntake();
        Gus.intake.rollerIn();
    }

    @Override
    public boolean isFinished() {
        if (follower.isFinished() || Gus.intake.isFull()) {
            follower.forceComplete();
            Gus.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Gus.intake.rollerStop();
    }
}

