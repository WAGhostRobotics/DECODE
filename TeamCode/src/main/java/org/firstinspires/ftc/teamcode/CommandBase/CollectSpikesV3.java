package org.firstinspires.ftc.teamcode.CommandBase;

import android.media.midi.MidiOutputPort;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Components.SimpleIntake;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class CollectSpikesV3 extends Command {
    MotionPlanner follower;
    public CollectSpikesV3(MotionPlanner follower) {
        this.follower = follower;
    }


    @Override
    public void init() {
        Bob.intake.rollerIn();
    }

    @Override
    public void update() {
//        if (Bob.intake.getCurrentDraw() >= SimpleIntake.currentThreshold) {
//            Bob.intake.rollerStop();
//        }
    }

    @Override
    public boolean isFinished() {
        if (follower.isFinished()) {
            Bob.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Bob.intake.rollerStop();
    }
}

