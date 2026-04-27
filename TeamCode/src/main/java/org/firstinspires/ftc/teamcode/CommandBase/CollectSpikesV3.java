package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class CollectSpikesV3 extends Command {
    MotionPlanner follower;
    public CollectSpikesV3(MotionPlanner follower) {
        this.follower = follower;
    }


    @Override
    public void init() {
        Walt.intake.closeGate();
        Walt.intake.setBallIn(false);
        Walt.intake.rollerIn();
    }

    @Override
    public void update() {
        Walt.intake.rollerIn();
        Walt.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (follower.isFinished() || Walt.intake.isFull()) {
            follower.forceComplete();
            Walt.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Walt.intake.rollerStop();
    }
}

