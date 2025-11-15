package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static double intakePower = 1;                 // Change this in dashboard at runtime
    public static double spinPower = 0.75;
    public static double targetVelocity = 0;
    public static double increment = 0.001;         // Change this in dashboard if you want to control speed with dpads
    public static double P = 0.125, I=0.00275, D = 0;
    public static double hoodPos = 0;

    public double currentVelocity, error;
    Servo rightHood, leftHood;
    DcMotorEx intake;
    CRServo s1, s2;
    RevColorSensorV3 colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");


        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Bob.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(intakePower);

            }
            else {
                s1.setPower(0);
                s2.setPower(0);
            }

            if (gamepad1.right_bumper) {
                intake.setPower(intakePower);
            }
            Bob.shooter.shoot();

            leftHood.setPosition(hoodPos);
            rightHood.setPosition(1-hoodPos);
            Bob.shooter.setPID(P, I, D);
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.intake.updateSpindexer();
            Bob.shooter.updateShooter();
            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("HoodPos: ", hoodPos);
            telemetry.update();

        }



    }
    private void rollerIn() {
        if (colorSensor.getDistance(DistanceUnit.CM)<5) {

        }
        intake.setPower(intakePower);
        s1.setPower(spinPower);
        s2.setPower(spinPower);
    }


}
