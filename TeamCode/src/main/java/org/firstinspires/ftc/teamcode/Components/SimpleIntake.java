package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SimpleIntake {
    DcMotorEx intake;
    DcMotorEx loader;
    Servo gate;
    public boolean gateOpen = true;
    double power;
    double outPower;
    public static final double currentThreshold = 4.1;
    boolean done;

    public SimpleIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        loader = hardwareMap.get(DcMotorEx.class, "loader");
        gate = hardwareMap.get(Servo.class, "gate");
        openGate();
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 1;
        outPower = 0.4;
    }

    public void rollerIn() {
        double current = getCurrentDraw();
        if (current > currentThreshold) {
            done = true;
        }
        if (!done) {
            intake.setPower(1);
            loader.setPower(1);
        }
        else {
            intake.setPower(1);
            loader.setPower(0);
        }
    }

    public void slowRollerIn() {
        intake.setPower(0.3);
    }

    public void rollerOut() {
        done = false;
        intake.setPower(-outPower);
    }

    public void rollerStop() {
        done = false;
        intake.setPower(0);
    }

    public void shoot() {
        intake.setPower(1);
        loader.setPower(1);
    }
    public void shootStop() {
        rollerStop();
        loader.setPower(0);
    }

    public void loaderStop() {
        loader.setPower(0);
    }

    public void setPower(double pw) {
        power = pw;
    }

    public void openGate() {
        gate.setPosition(0.9772);
        gateOpen = true;
    }

    public void closeGate() {
        gate.setPosition(0.3078);
        gateOpen = false;
    }

    public String getTelemetry() {
        return "Power: " + power;
    }

    public double getCurrentDraw() {
        return loader.getCurrent(CurrentUnit.AMPS);
    }

}
