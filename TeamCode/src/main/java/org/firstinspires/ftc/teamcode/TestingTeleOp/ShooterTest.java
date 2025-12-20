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



    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap, true, false);
        Bob.limelight.switchToGoalPipeline();

        waitForStart();
        while (opModeIsActive()) {
            Bob.localizer.update();
            Bob.limelight.trackAprilTag(Bob.localizer.getHeading(), Bob.shooter.getTurretAngle(), true);
            if (Bob.limelight.isVisible()) {
                Bob.shooter.setTurretTargetPos(Shooter.angleToPosition(Bob.limelight.getTurretAngle()));
//                Bob.shooter.setTurretTargetPos(targetPosition);
            }
            Bob.shooter.updateTurret();
            if (gamepad1.right_trigger>0.1) {
                Bob.intake.rollerIn();
            }
            else if (gamepad1.left_trigger>0.1) {
                Bob.intake.rollerOut();
            }
            else if (gamepad1.right_bumper) {
                Bob.shooter.shoot();
            }
            else {
                Bob.intake.rollerStop();
                Bob.shooter.stop();
            }

            Bob.shooter.setHood(hoodPos);
            Bob.shooter.setPID(P, I, D);
            Bob.shooter.setTargetVelocity(targetVelocity);
            Bob.shooter.updateShooter();

            telemetry.addData("Power: ", Bob.shooter.getTelemetry());
            telemetry.addData("Heading: ", Bob.localizer.getHeading());
            telemetry.addData("HoodPos: ", hoodPos);
            telemetry.update();

        }



    }
    private void rollerIn() {

    }


}
