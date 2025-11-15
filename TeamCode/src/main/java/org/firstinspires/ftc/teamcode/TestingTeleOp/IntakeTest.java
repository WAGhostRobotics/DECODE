package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    int currentPosition, error;
    public static int targetPosition;
    PIDController spinController;
    double power;
    double distance, prevDistance=10;

    boolean popUp = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, movementPower);
            if (gamepad1.right_trigger>0.3)
                Bob.intake.rollerIn();
            else
                Bob.intake.rollerStop();


            Bob.intake.setSpinPower(spinPower);
            Bob.intake.rotateCW();
//            distance = colorSensorV3.getDistance(DistanceUnit.CM);
//            if (distance <= 5) {
//                if (Bob.intake.isFinished()) {
//                    Bob.intake.nextSlot();
//                }
//            }
//            prevDistance = distance;
//            Bob.intake.updateSpindexer();

            telemetry.addData("CurrentPos: ", currentPosition);
            telemetry.addData("TargetPos: ", targetPosition);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("IsFinished: ", Bob.intake.isFinished());
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.update();
        }
    }
}
