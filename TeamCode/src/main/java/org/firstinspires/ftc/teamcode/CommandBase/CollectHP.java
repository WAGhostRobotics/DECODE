package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

public class CollectHP extends Command {
    Follower follower;
    double timerSeconds = 0;
    ElapsedTime timer;
    public CollectHP(Follower follower, double seconds) {
        this.follower = follower;
        timerSeconds = seconds;
        timer = new ElapsedTime();
    }


    @Override
    public void init() {
        timer.reset();
        Walt.intake.closeGate();
        Walt.intake.setBallIn(false);
        Walt.intake.rollerIn();
    }

    @Override
    public void update() {
        if (!PedroUtil.isFinished(follower)) {
            timer.reset();
        }
        Walt.intake.rollerIn();
        Walt.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() > timerSeconds || Walt.intake.isFull()) {
            follower.breakFollowing();
            Walt.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Walt.intake.rollerStop();
    }
}

