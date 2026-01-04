package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {

    public static double intakePower = 1;

    boolean popUp = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Bob.init(hardwareMap);
//        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));
            Bob.drivetrain.drive(magnitude, theta, driveTurn, 0.9);
            Bob.intake.setPower(intakePower);
            if (gamepad1.right_trigger>0.3)
                Bob.intake.rollerIn();
            else
                Bob.intake.rollerStop();


//            Bob.intake.setSpinPower(spinPower);
//            Bob.intake.rotateCW();
//            distance = colorSensorV3.getDistance(DistanceUnit.CM);
//            if (distance <= 5) {
//                if (Bob.intake.isFinished()) {
//                    Bob.intake.nextSlot();
//                }
//            }
//            prevDistance = distance;
//            Bob.intake.updateSpindexer();

//            telemetry.addData("IsFinished: ", Bob.intake.isFinished());
            telemetry.addData("Intake: ", Bob.intake.getTelemetry());
            telemetry.update();
        }
    }
}
