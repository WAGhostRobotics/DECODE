package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SimpleIntake {
    DcMotorEx intake;
    double power;
    double outPower;

    public SimpleIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 1;
        outPower = 0.4;
    }

    public void rollerIn() {
        intake.setPower(power);
    }

    public void rollerOut() {
        intake.setPower(-outPower);
    }

    public void rollerStop() {
        intake.setPower(0);
    }

    public void setPower(double pw) {
        power = pw;
    }

    public String getTelemetry() {
        return "Power: " + power;
    }

    public double getCurrentDraw() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

}
