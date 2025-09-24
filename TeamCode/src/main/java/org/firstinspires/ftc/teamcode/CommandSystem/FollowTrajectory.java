package org.firstinspires.ftc.teamcode.CommandSystem;


import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.AutoUtil.Path;

public class FollowTrajectory extends Command {
    MotionPlannerEdit mp;
    Path traj;

    public FollowTrajectory(MotionPlannerEdit mp, Path traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.resume();
        mp.startTrajectory(traj);
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
