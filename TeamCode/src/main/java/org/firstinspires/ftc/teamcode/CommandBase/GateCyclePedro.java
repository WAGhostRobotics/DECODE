package org.firstinspires.ftc.teamcode.CommandBase;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.PedroUtil;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

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
        Walt.intake.closeGate();
        Walt.intake.setBallIn(false);
        Walt.intake.rollerIn();
    }

    @Override
    public void update() {
        if (PedroUtil.isFinished(follower)) {
            finished = true;
        }
        if (!finished) {
            timer.reset();
        }
        Walt.intake.rollerIn();
        Walt.intake.updateIntake();
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds || Walt.intake.isFull()) {
            if (Walt.intake.isFull()) {
                Walt.intake.rollerStop();
                follower.breakFollowing();
            }
            Walt.intake.loaderStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }
}
