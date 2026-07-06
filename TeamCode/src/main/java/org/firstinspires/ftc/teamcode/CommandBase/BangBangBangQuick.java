package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class BangBangBangQuick extends Command {
    final double shootTime = 0.45; //Seconds
    Follower follower;
    double threshold;
    int targetVelocity;
    ElapsedTime timer;
    boolean ready;
    double shootThresh;
    public BangBangBangQuick(Follower follower, double threshold, int targetVelocity, double shootThresh) {
        timer = new ElapsedTime();
        this.follower = follower;
        this.threshold = threshold;
        this.targetVelocity = targetVelocity;
        this.shootThresh = shootThresh;

    }

    @Override
    public void init() {
        ready = false;
        timer.reset();
        Walt.shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void update() {
        if (follower.getCurrentTValue() >= (threshold -0.15)) {
            Walt.intake.rollerStop();
        }
        if (follower.getCurrentTValue() >= threshold) {
            Walt.intake.openGate();
        }
        if (Walt.shooter.reachedVelocity()) {
            ready = true;
        }

        if (follower.getCurrentTValue() > shootThresh && ready) {
            Walt.intake.shoot();
        }
        else {
            timer.reset();
        }

    }

    @Override
    public boolean isFinished() {
        if (follower.getCurrentTValue() > shootThresh && timer.seconds() > shootTime)
        {
            follower.breakFollowing();
            Walt.shooter.setTargetVelocity(0);
            Walt.shooter.stop();
            Walt.intake.rollerStop();
            Walt.intake.closeGate();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }


}
