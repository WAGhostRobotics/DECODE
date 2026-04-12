package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Gus;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gus.init(hardwareMap, false, true);
        ToggleButtonReader intakeReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        waitForStart();
        while (opModeIsActive()) {
            intakeReader.readValue();
            if (intakeReader.wasJustReleased()) {
                Gus.intake.setRampFullThreshold();
            }

            Gus.intake.closeGate();
            if (gamepad1.right_bumper) {
                Gus.intake.rollerIn();
            }
            else if (gamepad1.left_bumper) {
                Gus.intake.setBallIn(false);
                Gus.intake.rollerOut();
                Gus.intake.loaderStop();
            }
            else if (gamepad1.right_trigger > 0.1) {
                Gus.intake.shoot();
            }
            else {
                Gus.intake.rollerStop();
                Gus.intake.loaderStop();
            }
            Gus.intake.updateIntake();

            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
            telemetry.update();
        }

    }
}
