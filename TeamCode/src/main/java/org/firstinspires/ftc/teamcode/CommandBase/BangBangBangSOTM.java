package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class BangBangBangSOTM extends Command {
    final double shootTime = 0.5; //Seconds
    Follower follower;
    double threshold;
    int targetVelocity;
    ElapsedTime timer;
    public BangBangBangSOTM(Follower follower, double threshold, int targetVelocity) {
        timer = new ElapsedTime();
        this.follower = follower;
        this.threshold = threshold;
        this.targetVelocity = targetVelocity;

    }

    @Override
    public void init() {
        Walt.intake.openGate();
        timer.reset();
        Walt.shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void update() {
        if (follower.getCurrentTValue() >= threshold) {
            Walt.intake.shoot();
        }
        else {
            timer.reset();
        }

    }

    @Override
    public boolean isFinished() {
        if (follower.getCurrentTValue() >= threshold && timer.seconds() > shootTime)
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
