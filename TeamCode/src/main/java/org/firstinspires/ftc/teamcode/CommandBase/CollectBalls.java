package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class CollectBalls extends Command {
    ElapsedTime timer;
    MotionPlanner follower;
    boolean finished;
    double seconds;
    public CollectBalls(double seconds) {
        this.seconds = seconds;
        timer = new ElapsedTime();
    }


    @Override
    public void init() {
        finished = false;
        timer.reset();
        Walt.intake.closeGate();
        Walt.intake.setBallIn(false);
        Walt.intake.rollerIn();
    }

    @Override
    public void update() {
        if (follower.isFinished()) {
            finished = true;
        }
        if (!finished) {
            timer.reset();
        }
        Walt.intake.rollerIn();
        Walt.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds || Walt.intake.isFull()) {
            if (Walt.intake.isFull()) {
                Walt.intake.rollerStop();
                follower.forceComplete();
            }
            Walt.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }
}

