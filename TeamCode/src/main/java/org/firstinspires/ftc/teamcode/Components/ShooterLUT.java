package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.teamcode.Core.Bob;

public class ShooterLUT {
    public  InterpLUT speeds;
    public InterpLUT hoodAngle;

    public void init() {
        speeds = new InterpLUT();
        hoodAngle = new InterpLUT();
        hoodAngle.add(0, 0);
        hoodAngle.add(0.57, 0.4);
        hoodAngle.add(1.04, 0.35);
        hoodAngle.add(1.23, 0.2);
        hoodAngle.add(1.32, 0.19);
        hoodAngle.add(1.60, 0.17);
        hoodAngle.add(1.70, 0.18);
        hoodAngle.add(1.80, 0.18);
        hoodAngle.add(2.15, 0.12);
        hoodAngle.add(2.20, 0.12);
        hoodAngle.add(2.40, 0.12);
        hoodAngle.add(2.50, 0.12);
        hoodAngle.add(2.80, 0.0);
        hoodAngle.add(3.0, 0.0);
        hoodAngle.add(3.25, 0.0);
        hoodAngle.add(3.5, 0.0);
        hoodAngle.add(3.9, 0.0);
        hoodAngle.add(4.1, 0.0);
        hoodAngle.createLUT();

        speeds.add(0,0);
        speeds.add(0.57, 170);
        speeds.add(1.04, 170);
        speeds.add(1.23, 180);
        speeds.add(1.32, 181);
        speeds.add(1.60, 184);
        speeds.add(1.80, 185);
        speeds.add(2.0, 188);
        speeds.add(2.15, 189);
        speeds.add(2.20, 192);
        speeds.add(2.35, 194);

        speeds.add(2.40, 196);
        speeds.add(2.50, 196);
        speeds.add(2.80, 205);
        speeds.add(3.0, 208);
        speeds.add(3.25, 214);
        speeds.add(3.5, 219);
        speeds.add(3.9, 225);
        speeds.add(4.1, 227);
        speeds.createLUT();
    }

    public double getHoodAngle(double distance) {
        if (distance == 0 || distance>=4.1) {
            return Bob.shooter.getHoodPos();
        }
        return hoodAngle.get(distance);
    }

    public int getSpeed(double distance) {
        if (distance == 0 || distance>=4.1) {
            return (int) Bob.shooter.getTargetVelocity();
        }
        return (int) speeds.get(distance);
    }
}
