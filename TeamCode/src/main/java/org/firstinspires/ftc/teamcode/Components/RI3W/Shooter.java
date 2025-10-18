package org.firstinspires.ftc.teamcode.Components.RI3W;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private DcMotorEx intake;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private double P = 0.07, I=0.0035, D = 0;
    double currentVelocity, targetVelocity, error, power;
    public static double shootSpeed = 187;
    public static double farShootSpeed = 230;
    public static double intakeShootPower = 1;
    private final int standByVelocity = 100;

    private PIDController pidController;


    public void init(HardwareMap hardwareMap) {
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        targetVelocity = 0;
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    public boolean reachedVelocity() {
        return Math.abs(error)<3;
    }

    public double getCurrentVelocity() {
        currentVelocity = wheel1.getVelocity(AngleUnit.RADIANS) * 48; // mm
        return currentVelocity;
    }
    public void updateShooter() {
        getCurrentVelocity();
        error = targetVelocity - currentVelocity;
        power = pidController.calculate(0, error);
        power = Range.clip(power, -1, 1);
        wheel1.setPower(power);
        wheel2.setPower(power);
    }

    public void standBy() {
        setTargetVelocity(standByVelocity);
    }

    public void stop() {
        setTargetVelocity(0);
    }

    public void shoot() {
        if (reachedVelocity())
            intake.setPower(intakeShootPower);
        else
            intake.setPower(0);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }

    public void resetPID() {
        pidController.reset();
    }

    public String getTelemetry() {
        return "Target V: " + targetVelocity +
                "\nCurrent V: " + currentVelocity +
                "\nShooter Error: " + error +
                "\nPower: " + power;
    }
}
