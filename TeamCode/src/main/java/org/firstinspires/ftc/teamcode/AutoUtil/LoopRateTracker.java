package org.firstinspires.ftc.teamcode.AutoUtil;
public class LoopRateTracker {
    private long lastTime = System.nanoTime();
    private int loops = 0;
    private double loopRateHz = 0;

    // Call this once per loop
    public void updateLoopRate() {
        loops++;

        long now = System.nanoTime();
        if (now - lastTime >= 1_000_000_000L) { // 1 second
            loopRateHz = loops;   // loops per second
            loops = 0;
            lastTime = now;
        }
    }

    // Read the most recent loop rate
    public double getLoopRateHz() {
        return loopRateHz;
    }
}

