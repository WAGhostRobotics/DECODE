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
        hoodAngle.add(0, 0.9);
        hoodAngle.add(0.8, 0.9);
        hoodAngle.add(1.0, 0.7);
        hoodAngle.add(1.2, 0.5);
        hoodAngle.add(1.4, 0.53);
        hoodAngle.add(1.6, 0.46);
        hoodAngle.add(1.80, 0.4);
        hoodAngle.add(2.0, 0.36);
        hoodAngle.add(2.23, 0.34);
        hoodAngle.add(2.40, 0.34);
        hoodAngle.add(2.60, 0.32);
        hoodAngle.add(2.80, 0.25);
        hoodAngle.add(3.0, 0.23);
        hoodAngle.add(3.2, 0.23);
        hoodAngle.add(3.4, 0.22);
        hoodAngle.add(3.6, 0.22);
        hoodAngle.add(3.8, 0.2);
        hoodAngle.add(4.0, 0.18);
        hoodAngle.add(4.2, 0.19);
        hoodAngle.add(4.4, 0.18);
        hoodAngle.createLUT();

        speeds.add(0,0);
        speeds.add(0.8, 148);
        speeds.add(1.0, 155);
        speeds.add(1.2, 163);
        speeds.add(1.4, 164);
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
        speeds.add(3.6, 210);
        speeds.add(3.8, 216);
        speeds.add(4.0, 223);
        speeds.add(4.2, 226);
        speeds.add(4.4, 230);
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
