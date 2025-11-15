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
public class SpinTuner extends LinearOpMode {

    double power;
    double distance, prevDistance=10;
    public static int targetPosition = 0;

    public static double p=0.00016, i=0.000002, d;

    boolean popUp = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
        ToggleButtonReader aReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        waitForStart();
        while (opModeIsActive()) {
            Bob.intake.setPID(p, i, d);
            aReader.readValue();
            if (aReader.wasJustReleased()) {
                targetPosition = targetPosition +Intake.slotIncrement;
            }
            Bob.intake.setTargetPosition(targetPosition);
            Bob.intake.updateSpindexer();

            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.update();
        }
    }
}
