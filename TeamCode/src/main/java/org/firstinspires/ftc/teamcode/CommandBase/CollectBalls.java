package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

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
        Gus.intake.closeGate();
        Gus.intake.setBallIn(false);
        Gus.intake.rollerIn();
    }

    @Override
    public void update() {
        if (!follower.isFinished()) {
            timer.reset();
        }
        Gus.intake.updateIntake();
        Gus.intake.rollerIn();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds || Gus.intake.isFull()) {
            if (Gus.intake.isFull()) {
                Gus.intake.rollerStop();
                follower.forceComplete();
            }
            Gus.intake.slowRollerIn();
            Gus.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Gus.intake.rollerStop();
    }
}

