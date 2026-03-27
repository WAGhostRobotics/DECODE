package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class BangBangBang extends Command {
    final double shootTime = 0.5; //Seconds
    Follower follower;
    double threshold;
    int targetVelocity;
    ElapsedTime timer;
    public BangBangBang(Follower follower, double threshold, int targetVelocity) {
        timer = new ElapsedTime();
        this.follower = follower;
        this.threshold = threshold;
        this.targetVelocity = targetVelocity;

    }

    @Override
    public void init() {
        timer.reset();
        Gus.shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void update() {
        if (follower.getCurrentTValue() >= threshold) {
            Gus.intake.openGate();
        }

        if (PedroUtil.isFinished(follower)) {
            Gus.intake.shoot();
        }
        else {
            timer.reset();
        }

    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() > shootTime)
        {
            Gus.shooter.setTargetVelocity(0);
            Gus.shooter.stop();
            Gus.intake.rollerStop();
            Gus.intake.closeGate();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }


}
