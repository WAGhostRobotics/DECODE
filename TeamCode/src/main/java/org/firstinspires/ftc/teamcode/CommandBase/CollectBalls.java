package org.firstinspires.ftc.teamcode.CommandBase;

import android.media.midi.MidiOutputPort;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Components.SimpleIntake;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class CollectBalls extends Command {
    ElapsedTime timer;
    MotionPlanner follower;
    double seconds;
    public CollectBalls(MotionPlanner follower, double seconds) {
        this.follower = follower;
        this.seconds = seconds;
        timer = new ElapsedTime();
    }


    @Override
    public void init() {
        timer.reset();
        Bob.intake.closeGate();
        Bob.intake.setBallIn(false);
        Bob.intake.rollerIn();
    }

    @Override
    public void update() {
        if (!follower.isFinished()) {
            timer.reset();
        }
        Bob.intake.updateIntake();
        Bob.intake.rollerIn();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds || Bob.intake.isFull()) {
            if (Bob.intake.isFull()) {
                Bob.intake.rollerStop();
                follower.forceComplete();
            }
            Bob.intake.slowRollerIn();
            Bob.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Bob.intake.rollerStop();
    }
}

