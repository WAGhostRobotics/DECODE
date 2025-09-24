package org.firstinspires.ftc.teamcode.RI3W;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;

public class George {
    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static double movementPower;
    public static PinpointLocalizer localizer;
    public static DcMotorEx intake;
    public static DcMotorEx wheel1;
    public static DcMotorEx wheel2;

    public static void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        George.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;

    }


}
