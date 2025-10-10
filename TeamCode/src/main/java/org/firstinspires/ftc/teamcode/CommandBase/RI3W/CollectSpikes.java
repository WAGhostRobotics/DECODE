package org.firstinspires.ftc.teamcode.CommandBase.RI3W;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.RI3W.George;

public class CollectSpikes extends Command {
    ElapsedTime timer;
    double movementPower, seconds, intakePower;
    public CollectSpikes(double movementPower, double seconds, double intakePower) {
        timer = new ElapsedTime();
        this.movementPower = movementPower;
        this.seconds = seconds;
        this.intakePower = intakePower;
    }

    @Override
    public void init() {
        George.shooter.setIntake(intakePower);
        timer.reset();
        George.drivetrain.drive(1, 0, 0, movementPower);
    }

    @Override
    public void update() {
        if (timer.seconds() >= seconds) {
            George.drivetrain.drive(0,0,0,0);
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            George.drivetrain.drive(0, 0, 0, 0);
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        George.drivetrain.drive(0,0,0,0);
    }
}
