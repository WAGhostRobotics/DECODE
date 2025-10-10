package org.firstinspires.ftc.teamcode.RI3W;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Components.RI3W.Camera;
import org.firstinspires.ftc.teamcode.Components.RI3W.Shooter;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.MecanumDrive;

public class George {
    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static Camera limelight;
    public static double movementPower;
    public static PinpointLocalizer localizer;

    public static Shooter shooter;

    public static void init(HardwareMap hardwareMap) {

        shooter = new Shooter();
        shooter.init(hardwareMap);
        George.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        limelight = new Camera(hardwareMap);

    }

}
