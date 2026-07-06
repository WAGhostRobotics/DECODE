package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    Servo lift1, lift2;
    private double pos;
    private final double liftPos = 0.55;
    private final double brakePos = 0.673;
    private final double retractPos = 0.687;

    public void init(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
    }

    public void setLift(double pos) {
        this.pos = pos;
        lift1.setPosition(pos);
        lift2.setPosition(1-pos);
    }

    public double getPos() {
        return pos;
    }

    public void lift() {
        setLift(liftPos);
    }

    public void retract() {
        setLift(retractPos);
    }

    public void brake() {
        setLift(brakePos);
    }
}
