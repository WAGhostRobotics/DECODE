package org.firstinspires.ftc.teamcode.TestingTeleOp.RI3W;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotorEx intake;
    Gamepad gamepad;
    public Intake(HardwareMap hardwareMap, Gamepad gamepad){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        this.gamepad = gamepad;
    }

    public void control(){
        if (gamepad.dpad_up){
            intake.setPower(intake.getPower() + 0.001);
        }else if(gamepad.dpad_down){
            intake.setPower(intake.getPower() - 0.001);
        }
    }

    public void zeroPower(){
        intake.setPower(0);
    }
}
