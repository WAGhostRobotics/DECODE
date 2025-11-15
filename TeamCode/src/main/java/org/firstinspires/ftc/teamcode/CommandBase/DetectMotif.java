package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class DetectMotif extends Command {
    ElapsedTime timer;
    double seconds;
    public DetectMotif(double seconds) {
        timer = new ElapsedTime();
        this.seconds = seconds;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void update() {
        Bob.intake.setMotif(Bob.limelight.getMotif());
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= seconds;
    }

    @Override
    public void stop() {

    }
}
