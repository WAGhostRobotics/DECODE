package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Bob;

public class RobotTest extends LinearOpMode {
    boolean shooterOn = false;
    public static int targetVelocity = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        Bob.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger>0.1) {
                if (gamepad1.right_bumper) {
                    Bob.intake.rotateCW();
                }
                Bob.intake.rollerIn();
            }
            else if (gamepad1.left_bumper) {
                Bob.intake.rollerOut();
            }

            if (shooterOn) {
                Bob.shooter.setTargetVelocity(targetVelocity);
                if (gamepad1.right_trigger > 0.1) {
                }
            }
            else {
                Bob.shooter.stop();
            }


            if (shooterButton.wasJustReleased()) {
                shooterOn = !shooterOn;
            }
            shooterButton.readValue();
        }
    }
}
