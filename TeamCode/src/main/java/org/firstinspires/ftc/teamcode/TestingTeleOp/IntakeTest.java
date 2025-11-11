package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {

    DcMotorEx intake;
    CRServo s1, s2;
    public static double targetVelocity = 0;
    public static double P = 0.125, I=0.00275, D = 0;
    public static double hoodPos = 0;
    public static double intakePower = 0.8;
    public static double spinPower = 0.75;
    public static double movementPower = 0.45;
    boolean popUp = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, movementPower);

            intake.setPower(intakePower);
            s1.setPower(spinPower);
            s2.setPower(spinPower);

            telemetry.addData("Index: ", (int)Math.round(Math.abs((double)intake.getCurrentPosition()/Intake.slotIncrement)) %3);
            telemetry.update();
        }
    }
}
