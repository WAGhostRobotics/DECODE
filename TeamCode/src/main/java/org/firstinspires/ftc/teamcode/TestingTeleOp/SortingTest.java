package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.G;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.P;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class SortingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader aReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader bReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        Intake.SlotState[] motif = new Intake.SlotState[] {P, P, G};
        Bob.init(hardwareMap);
        Bob.limelight.switchToMotifPipeline();
        waitForStart();
        while (opModeIsActive()) {
            Bob.intake.setMotif(Bob.limelight.getMotif());
            aReader.readValue();
            bReader.readValue();
            if (aReader.wasJustReleased()) {
                Bob.intake.holdAtZero();
            }
            if (gamepad1.right_trigger>0.2) {
                Bob.intake.autoIntake();
                Bob.intake.rollerIn();
            }
            else if (gamepad1.left_trigger>0.2) {
                Bob.intake.rollerOut();
            }
            else {
                Bob.intake.rollerStop();
            }

            if (bReader.wasJustReleased()) {
                Bob.intake.sort();
            }

            Bob.intake.updateSpindexer();

            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.addData("LL Motif: ", Bob.limelight.getMotif());
            telemetry.update();
        }
    }
}
