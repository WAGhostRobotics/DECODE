package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.teamcode.Core.Gus;

public class ShooterLUT {
    public  InterpLUT speeds;
    public InterpLUT hoodAngle;
    public InterpLUT timeInAir;

    public void init() {
        speeds = new InterpLUT();
        hoodAngle = new InterpLUT();
        timeInAir = new InterpLUT();
        hoodAngle.add(0, 0.95);
        hoodAngle.add(0.8, 0.95);
        hoodAngle.add(1.0, 0.82);
        hoodAngle.add(1.2, 0.59);
        hoodAngle.add(1.4, 0.5);
        hoodAngle.add(1.6, 0.41);
        hoodAngle.add(1.80, 0.36);
        hoodAngle.add(2.0, 0.32);
        hoodAngle.add(2.23, 0.31);
        hoodAngle.add(2.40, 0.32);
        hoodAngle.add(2.60, 0.30);//
        hoodAngle.add(2.80, 0.25);
        hoodAngle.add(3.0, 0.20);
        hoodAngle.add(3.22, 0.16);
        hoodAngle.add(3.4, 0.12);//
        hoodAngle.add(3.6, 0.10);
        hoodAngle.add(3.8, 0.06);//
        hoodAngle.add(4.0, 0.06);
        hoodAngle.add(4.2, 0.06);
        hoodAngle.add(4.4, 0.06);
        hoodAngle.createLUT();

        speeds.add(0,0);
        speeds.add(0.8, 131);
        speeds.add(1.0, 137);
        speeds.add(1.2, 140);
        speeds.add(1.4, 158);
        speeds.add(1.60, 166);
        speeds.add(1.80, 171);
        speeds.add(2.0, 174);
        speeds.add(2.23, 177);
        speeds.add(2.4, 178);
        speeds.add(2.6, 184);
        speeds.add(2.8, 195);
        speeds.add(3.0, 200);
        speeds.add(3.20, 202);
        speeds.add(3.4, 207);
        speeds.add(3.6, 212);
        speeds.add(3.8, 212);
        speeds.add(4.0, 212);
        speeds.add(4.2, 214);
        speeds.add(4.4, 215);
        speeds.createLUT();

        timeInAir.add(0, 0);
        timeInAir.add(0.8, 0.7);
        timeInAir.add(1.0, 0.7);
        timeInAir.add(1.2, 0.7);
        timeInAir.add(1.4, 0.7);
        timeInAir.add(1.6, 0.7);
        timeInAir.add(1.80, 0.7);
        timeInAir.add(2.0, 0.75);
        timeInAir.add(2.23, 0.8);
        timeInAir.add(2.40, 0.85);
        timeInAir.add(2.60, 0.85);
        timeInAir.add(2.80, 0.85);
        timeInAir.add(3.0, 0.85);
        timeInAir.add(3.2, 0.8);
        timeInAir.add(3.4, 0.8);
        timeInAir.add(3.6, 0.8);
        timeInAir.add(3.8, 0.8);
        timeInAir.add(4.2, 0.9);
        timeInAir.add(4.4, 0.9);
        timeInAir.createLUT();
    }

    public double getHoodAngle(double distance) {
        if (distance == 0 || distance>=4.1) {
            return Gus.shooter.getHoodPos();
        }
        return hoodAngle.get(distance);
    }

    public int getSpeed(double distance) {
        if (distance == 0 || distance>=4.1) {
            return (int) Gus.shooter.getTargetVelocity();
        }
        return (int) speeds.get(distance);
    }

    public double getAirTime(double distance) {
        if (distance == 0 || distance>=4.1) {
            return 0;
        }
        return timeInAir.get(distance);
    }
}
