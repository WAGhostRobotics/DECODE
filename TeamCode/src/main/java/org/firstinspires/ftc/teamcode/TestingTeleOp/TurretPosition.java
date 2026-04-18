package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Shooter;

@Config
@TeleOp
public class TurretPosition extends OpMode {

    Servo turret1;
    Servo turret2;
    public static double pos = 0.5;
    public static double backlashIncrement = 0;
    public static double angle = 0;
    @Override
    public void init() {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

    }

    @Override
    public void loop() {
//        pos = Shooter.angleToPosition(angle);
        turret1.setPosition(pos + backlashIncrement);
        turret2.setPosition(pos - backlashIncrement);

        telemetry.addData("Position: ", pos);
        telemetry.update();
    }
}
