package org.firstinspires.ftc.teamcode.CommandSystem;

/**
 * @author Snakuwul Joshi
 * **/

public class RunCommand extends Command {

    RunFunction func;

    public RunCommand(RunFunction function){
        this.func = function;
    }

    @Override
    public void init() {
        func.run();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void stop() {}
}
