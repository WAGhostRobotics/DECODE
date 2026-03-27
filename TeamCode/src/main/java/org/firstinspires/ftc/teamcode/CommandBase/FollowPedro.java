package org.firstinspires.ftc.teamcode.CommandBase;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Path;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class FollowPedro extends Command {
    PathChain traj;
    Follower mp;
    public FollowPedro(Follower mp, PathChain traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.followPath(traj);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return PedroUtil.isFinished(mp);
    }

    @Override
    public void stop() {}
}
