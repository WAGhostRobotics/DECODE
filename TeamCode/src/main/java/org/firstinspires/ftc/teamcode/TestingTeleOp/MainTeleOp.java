package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.TeleShoot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Bob;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    boolean blue = true;
    public static double delay = 1;
    boolean shooterOn = false;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        ToggleButtonReader zoneReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);


        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);


        Bob.init(hardwareMap, false, true);

        while (opModeInInit()) {
            zoneReader.readValue();
            if (zoneReader.wasJustReleased()) {
                blue = !blue;
                Bob.limelight.setBlueAlliance(blue);
            }
            telemetry.addData("Blue: ", blue);
            telemetry.update();
        }
        waitForStart();
        Bob.intake.closeGate();

        Bob.limelight.switchToGoalPipeline();
        while (opModeIsActive()) {
            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading(), Bob.shooter.getTurretAngle(), true);
            double distance = Bob.limelight.getDistance();

            Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));

            Bob.shooter.updateTurret();
            if (gateReader.wasJustReleased()) {
                Bob.intake.setBallIn(false);
                Bob.intake.closeGate();
            }

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            double heading = Bob.localizer.getHeading();
            theta = normalizeDegrees(theta - heading);
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
            Bob.intake.updateIntake();

            if (gamepad1.left_trigger>0.1) {
                Bob.intake.rollerOut();
            }
            else if (gamepad1.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    Bob.intake.loaderStop();
                    Bob.intake.rollerStop();
                    Bob.intake.setBallIn(true);
                    Bob.intake.openGate();
                }
                else if (timer.seconds() > delay) {
                    Bob.shooter.shoot();
                }
                else {
                    Bob.intake.rollerStop();
                    Bob.shooter.stop();
                }
            }
            else if (shootButton.wasJustReleased()) {
                Bob.intake.loaderStop();
            }
            else {
                timer.reset();
                if (!Bob.intake.gateOpen)
                    Bob.intake.rollerIn();
                else
                    Bob.intake.rollerStop();
            }

            Bob.shooter.setHood(Bob.shooterLUT.getHoodAngle(distance));



            if (Bob.intake.isOneBallIn()) {
                Bob.shooter.setTargetVelocity(Bob.shooterLUT.getSpeed(distance));
            }
            else {
                Bob.shooter.setTargetVelocity(0);
            }
            Bob.shooter.updateShooter();

            gateReader.readValue();
            zoneReader.readValue();
            shootButton.readValue();
            imuReader.readValue();

            if (zoneReader.wasJustReleased()) {
                blue = !blue;
                Bob.limelight.setBlueAlliance(blue);
                Bob.limelight.switchToGoalPipeline();
            }

            if (imuReader.wasJustReleased()) {
                Bob.localizer.resetHeading();
            }


            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Distance: ", Bob.limelight.getDistance());
            telemetry.addData("X: ", Bob.localizer.getPosX());
            telemetry.addData("Y: ", Bob.localizer.getPosY());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();

        }



    }
    private void rollerIn() {

    }


}
