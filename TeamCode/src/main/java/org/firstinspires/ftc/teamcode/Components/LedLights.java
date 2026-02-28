package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LedLights {
    RevBlinkinLedDriver ledDriver;

    public LedLights(HardwareMap hardwareMap) {
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");
    }

    public void turnOff() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public void orange() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void green() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
    }

    public void red() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
    }
    public void blue() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }

}
