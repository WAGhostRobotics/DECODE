package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Bob;

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
        Bob.shooter.setIntake(intakePower);
        timer.reset();
        Bob.drivetrain.drive(1, 0, 0, movementPower);
    }

    @Override
    public void update() {
        if (timer.seconds() >= seconds) {
            Bob.drivetrain.drive(0,0,0,0);
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= seconds) {
            Bob.drivetrain.drive(0, 0, 0, 0);
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        Bob.drivetrain.drive(0,0,0,0);
    }
}
