package org.firstinspires.ftc.teamcode.OldStuff;


import org.firstinspires.ftc.teamcode.AutoUtil.Path;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class FollowTrajectoryOld extends Command {
    MotionPlannerOld mp;
    Path traj;

    public FollowTrajectoryOld(MotionPlannerOld mp, Path traj) {
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
