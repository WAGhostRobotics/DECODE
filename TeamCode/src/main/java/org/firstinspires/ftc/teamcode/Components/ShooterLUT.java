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
        hoodAngle.add(0, 0.98);
        hoodAngle.add(0.6, 0.98);
        hoodAngle.add(0.8, 0.85);
        hoodAngle.add(1.0, 0.78);
        hoodAngle.add(1.2, 0.65);
        hoodAngle.add(1.4, 0.6);
        hoodAngle.add(1.6, 0.53);
        hoodAngle.add(1.80, 0.48);
        hoodAngle.add(2.0, 0.45);
        hoodAngle.add(2.2, 0.43);
        hoodAngle.add(2.40, 0.43);
        hoodAngle.add(2.60, 0.4);//
        hoodAngle.add(2.80, 0.38);
        hoodAngle.add(3.0, 0.38);
        hoodAngle.add(3.2, 0.38);
        hoodAngle.add(3.4, 0.34);//
        hoodAngle.add(3.6, 0.32);
        hoodAngle.add(3.8, 0.32);//
//        hoodAngle.add(4.0, 0.1);
//        hoodAngle.add(4.2, 0.1);
//        hoodAngle.add(4.4, 0.1);
        hoodAngle.createLUT();

        speeds.add(0,0);
        speeds.add(0.6, 75);//
        speeds.add(0.8, 79);//
        speeds.add(1.0, 82);//
        speeds.add(1.2, 83);//
        speeds.add(1.4, 87);//
        speeds.add(1.60, 94);//
        speeds.add(1.80, 98);//
        speeds.add(2.0, 103); //
        speeds.add(2.2, 108);
        speeds.add(2.4, 110);
        speeds.add(2.6, 115);
        speeds.add(2.8, 118);
        speeds.add(3.0, 120);
        speeds.add(3.2, 122);

        speeds.add(3.4, 124);
        speeds.add(3.6, 128); // 0.14 203
        speeds.add(3.8, 133);
//        speeds.add(4.0, 212);
//        speeds.add(4.2, 221);
//        speeds.add(4.4, 224);
        speeds.createLUT();

        timeInAir.add(0, 0);
        timeInAir.add(0.6, 0.7);
        timeInAir.add(0.8, 0.7);
        timeInAir.add(1.0, 0.7);
        timeInAir.add(1.2, 0.7);
        timeInAir.add(1.4, 0.7);
        timeInAir.add(1.6, 0.7);
        timeInAir.add(1.8, 0.7);
        timeInAir.add(2.0, 0.75);
        timeInAir.add(2.20, 0.75);
        timeInAir.add(2.40, 0.8);
        timeInAir.add(2.60, 0.85);
        timeInAir.add(2.8, 0.85);
        timeInAir.add(3.0, 0.8);
        timeInAir.add(3.2, 0.8);
        timeInAir.add(3.4, 0.8);
        timeInAir.add(3.6, 0.8);
        timeInAir.add(3.8, 0.85);
        timeInAir.createLUT();
    }

    public double getHoodAngle(double distance) {
        if (distance == 0 || distance>=3.8) {
            if (distance >= 3.8) {
                return 0.3;
            }
            return 0.98;
        }
        return hoodAngle.get(distance);
    }

    public int getSpeed(double distance) {
        if (distance == 0 || distance>=3.8) {
            if (distance >= 3.8) {
                return 133;
            }
            return (int) Gus.shooter.getTargetVelocity();
        }
        return (int) speeds.get(distance);
    }

    public double getAirTime(double distance) {
        if (distance == 0 || distance>=3.8) {
            return 0;
        }
        return timeInAir.get(distance);
    }
}
