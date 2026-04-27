package org.firstinspires.ftc.teamcode.CommandBase;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class GateCollectPedro extends Command {
    Follower follower;
    boolean finished;
    double seconds = 1.2;
    ElapsedTime timer;
    Path[] paths;
    int i = 0;
    public GateCollectPedro(double seconds, Follower follower, Path... paths) {
        this.seconds = seconds;
        timer = new ElapsedTime();
        this.follower = follower;
        this.paths = paths;
    }


    @Override
    public void init() {
        Walt.intake.setBallIn(false);
        finished = false;
        follower.followPath(paths[0], true);
    }

    @Override
    public void update() {
        if (PedroUtil.isFinished(follower)) {
            if (timer.seconds() > seconds) {
                i++;
                if (i < paths.length) {
                    follower.followPath(paths[i], true);
                }
                else {
                    finished = true;
                }
            }
//            i++;
//            if (i < paths.length) {
//                follower.breakFollowing();
//                follower.followPath(paths[i], true);
//            }
//            else {
//                finished = true;
//            }
        }
        else {
            timer.reset();
        }
        Walt.intake.updateIntake();
        Walt.intake.rollerIn();
    }

    @Override
    public boolean isFinished() {
        if (Walt.intake.isFull() || (finished)) {
            Walt.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }
}
