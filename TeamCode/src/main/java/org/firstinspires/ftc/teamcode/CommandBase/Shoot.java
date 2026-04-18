package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class Shoot extends Command {
    ElapsedTime timer;
    double seconds;
    boolean ready;
    public Shoot(double seconds) {
        timer = new ElapsedTime();
        this.seconds = seconds;
    }

    @Override
    public void init() {
        timer.reset();
        ready = false;
        Gus.intake.openGate();
    }

    @Override
    public void update() {
        if (Gus.shooter.reachedVelocity()) {
            ready = true;
        }
        if (ready) {
            Gus.intake.shoot();
        }
        else {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            Gus.shooter.stop();
            Gus.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }



}
