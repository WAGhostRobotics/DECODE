package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TurretPosition extends OpMode {

    DcMotorEx wheel1;
    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel2");
    }

    @Override
    public void loop() {
        telemetry.addData("Position: ", wheel1.getCurrentPosition());
        telemetry.update();
    }
}
