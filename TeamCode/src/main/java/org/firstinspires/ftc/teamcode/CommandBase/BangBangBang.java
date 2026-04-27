package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class BangBangBang extends Command {
    final double shootTime = 0.4; //Seconds
    Follower follower;
    double threshold;
    int targetVelocity;
    ElapsedTime timer;
    boolean ready;
    public BangBangBang(Follower follower, double threshold, int targetVelocity) {
        timer = new ElapsedTime();
        this.follower = follower;
        this.threshold = threshold;
        this.targetVelocity = targetVelocity;

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

        if (PedroUtil.isFinished(follower) && ready) {
            Walt.intake.shoot();
        }
        else {
            timer.reset();
        }

    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() > shootTime)
        {
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
