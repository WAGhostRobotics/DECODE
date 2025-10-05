package org.firstinspires.ftc.teamcode.CommandBase;


import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Path;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class FollowTrajectory extends Command {
    MotionPlanner mp;
    Path traj;

    public FollowTrajectory(MotionPlanner mp, Path traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.resume();
        mp.startFollowingPath(traj);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mp.isFinished();
    }

    @Override
    public void stop() {}
}
