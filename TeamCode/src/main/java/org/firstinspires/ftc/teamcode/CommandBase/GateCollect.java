package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class GateCollect extends Command {
    MotionPlanner follower;
    boolean finished;
    double seconds;
    Bezier[] paths;
    int i = 0;
    public GateCollect(MotionPlanner follower, Bezier ... paths) {
        this.follower = follower;
        this.paths = paths;
    }


    @Override
    public void init() {
        follower.resume();
        finished = false;
        follower.startFollowingPath(paths[0]);
    }

    @Override
    public void update() {
        if (follower.isFinished()) {
            i++;
            if (i < paths.length) {
                follower.startFollowingPath(paths[i]);
            }
        }
        Gus.intake.updateIntake();
        Gus.intake.rollerIn();
    }

    @Override
    public boolean isFinished() {
        if (Gus.intake.isFull() || (follower.isFinished() && i >= paths.length)) {
            Gus.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }
}
