package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Walt;

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
        Walt.intake.openGate();
    }

    @Override
    public void update() {
        if (Walt.shooter.reachedVelocity()) {
            ready = true;
        }
        if (ready) {
            Walt.intake.shoot();
        }
        else {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            Walt.shooter.stop();
            Walt.intake.rollerStop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }



}
