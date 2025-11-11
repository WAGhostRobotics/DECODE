package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static double power = 0.8;                 // Change this in dashboard at runtime
    public static double intakePower = 0;
    public static double targetVelocity = 0;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public static double P = 0.125, I=0.00275, D = 0;
    public static double hoodPos = 0;

    public double currentVelocity, error;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.get(Servo.class, "hood");
        Bob.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            hood.setPosition(hoodPos);
            Bob.shooter.setPID(P, I, D);
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.updateShooter();


            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.update();

        }
    }
}
