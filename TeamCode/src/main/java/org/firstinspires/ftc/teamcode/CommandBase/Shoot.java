package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class Shoot extends Command {
    ElapsedTime timer;
    double seconds;
    public Shoot(double seconds) {
        timer = new ElapsedTime();
        this.seconds = seconds;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void update() {
        if (timer.seconds() <= seconds) {
            Gus.shooter.shoot();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            Gus.shooter.stop();
            return true;
        }
        return false;
    }

    @Override
    public void stop() {

    }



}
