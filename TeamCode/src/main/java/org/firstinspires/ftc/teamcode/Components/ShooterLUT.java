package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Core.Walt;

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
        hoodAngle.add(1.4, 0.615);
        hoodAngle.add(1.6, 0.55);
        hoodAngle.add(1.80, 0.48);
        hoodAngle.add(2.0, 0.47);
        hoodAngle.add(2.2, 0.415);
        hoodAngle.add(2.40, 0.43);
        hoodAngle.add(2.60, 0.4);//
        hoodAngle.add(2.80, 0.4);
        hoodAngle.add(3.0, 0.425);
        hoodAngle.add(3.2, 0.415);
        hoodAngle.add(3.3, 0.41);
        hoodAngle.add(3.4, 0.42);//66
        hoodAngle.add(3.6, 0.37);
        hoodAngle.add(3.8, 0.36);//
//        hoodAngle.add(4.0, 0.1);
//        hoodAngle.add(4.2, 0.1);
//        hoodAngle.add(4.4, 0.1);
        hoodAngle.createLUT();

        speeds.add(0,0);
        speeds.add(0.6, 108);//
        speeds.add(0.8, 113);//
        speeds.add(1.0, 119);//
        speeds.add(1.2, 120);//
        speeds.add(1.4, 126);//
        speeds.add(1.60, 137);//
        speeds.add(1.80, 144);//
        speeds.add(2.0, 148); //
        speeds.add(2.2, 155);

        speeds.add(2.4, 164);
        speeds.add(2.6, 166);
        speeds.add(2.8, 177);
        speeds.add(3.0, 177);
        speeds.add(3.2, 182);
        speeds.add(3.3, 184);
        speeds.add(3.4, 185);
        speeds.add(3.5, 193);
        speeds.add(3.6, 195); // 0.14 203
        speeds.add(3.8, 197);
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
            return (int) Walt.shooter.getTargetVelocity();
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
