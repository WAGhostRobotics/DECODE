package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.MecanumDrive;

public class Bob {
    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static Camera limelight;
    public static double movementPower;
    public static PinpointLocalizer localizer;

    public static Shooter shooter;
    public static Intake intake;

    public static void init(HardwareMap hardwareMap) {

        shooter = new Shooter();
        shooter.init(hardwareMap);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new Intake(hardwareMap);
        limelight = new Camera(hardwareMap);

    }

    public static void init(HardwareMap hardwareMap, boolean blueAlliance) {

        shooter = new Shooter();
        shooter.init(hardwareMap);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new Intake(hardwareMap);
        limelight = new Camera(hardwareMap, blueAlliance);

    }

    public static void init(HardwareMap hardwareMap, boolean blueAlliance, boolean teleop) {

        shooter = new Shooter();
        shooter.init(hardwareMap, teleop);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new Intake(hardwareMap, teleop);
        limelight = new Camera(hardwareMap, blueAlliance);

    }

}
