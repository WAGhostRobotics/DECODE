package org.firstinspires.ftc.teamcode.RI3W;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    public static DcMotorEx intake;
    public static DcMotorEx wheel1;
    public static DcMotorEx wheel2;
    public static double P = 0.01, I=0, D = 0;
    double currentVelocity, targetVelocity, error, power;
    public static double shootSpeed = 187;
    public static double farShootSpeed = 230;

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

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

}
