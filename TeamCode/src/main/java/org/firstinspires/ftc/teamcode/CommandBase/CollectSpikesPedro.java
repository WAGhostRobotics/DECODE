package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class CollectSpikesPedro extends Command {
    Follower follower;
    public CollectSpikesPedro(Follower follower) {
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
        Gus.intake.rollerIn();
        Gus.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (PedroUtil.isFinished(follower) || Gus.intake.isFull()) {
            follower.breakFollowing();
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

