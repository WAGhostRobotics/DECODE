package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class GateCyclePedro extends Command {
    ElapsedTime timer;
    Follower follower;
    boolean finished;
    double seconds;
    public GateCyclePedro(Follower follower, double seconds) {
        this.follower = follower;
        this.seconds = seconds;
        timer = new ElapsedTime();
    }


    @Override
    public void init() {
        finished = false;
        timer.reset();
        Gus.intake.closeGate();
        Gus.intake.setBallIn(false);
        Gus.intake.rollerIn();
    }

    @Override
    public void update() {
        if (PedroUtil.isFinished(follower)) {
            finished = true;
        }
        if (!finished) {
            timer.reset();
        }
        Gus.intake.rollerIn();
        Gus.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds || Gus.intake.isFull()) {
            if (Gus.intake.isFull()) {
                Gus.intake.rollerStop();
                follower.breakFollowing();
            }
            Gus.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }
}
