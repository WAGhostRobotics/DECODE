package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Components.ShooterLUT;
import org.firstinspires.ftc.teamcode.Components.SimpleIntake;

public class Bob {
    public static ShooterLUT shooterLUT;
    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static Camera limelight;
    public static double movementPower;
    public static PinpointLocalizer localizer;

    public static Shooter shooter;
    public static SimpleIntake intake;

    public static void init(HardwareMap hardwareMap) {
        shooterLUT = new ShooterLUT();
        shooterLUT.init();
        shooter = new Shooter();
        shooter.init(hardwareMap);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new SimpleIntake(hardwareMap);
        limelight = new Camera(hardwareMap);
        shooter.setHood(0);

    }

    public static void init(HardwareMap hardwareMap, boolean blueAlliance) {
        shooterLUT = new ShooterLUT();
        shooterLUT.init();

        shooter = new Shooter();
        shooter.init(hardwareMap);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new SimpleIntake(hardwareMap);
        limelight = new Camera(hardwareMap, blueAlliance);
        shooter.setHood(0);


    }

    public static void init(HardwareMap hardwareMap, boolean blueAlliance, boolean teleop) {
        shooterLUT = new ShooterLUT();
        shooterLUT.init();

        shooter = new Shooter();
        shooter.init(hardwareMap, teleop);
        Bob.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrive(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
        movementPower = 0.8;
        intake = new SimpleIntake(hardwareMap);
        limelight = new Camera(hardwareMap, blueAlliance);
        shooter.setHood(0);


    }

}
