package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {

    public static double intakePower = 1;
    public static double distanceThreshold = 5;
    public static double currentThreshold = 5;

    boolean popUp = false;
    boolean done = false;
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 distanceSensor = hardwareMap.get(RevColorSensorV3.class, "distance");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx loader = hardwareMap.get(DcMotorEx.class, "loader");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            double magnitude = Math.hypot(x, y);
            double theta = Math.toDegrees(Math.atan2(y, x));

            double current = loader.getCurrent(CurrentUnit.AMPS);
            if (gamepad1.right_bumper) {
                gate.setPosition(0.2717);
                if (current > currentThreshold) {
                    done = true;
                }
                if (!done) {
                    intake.setPower(1);
                    loader.setPower(1);
                }
                else {
                    intake.setPower(1);
                    loader.setPower(0);
                }
            }
            else if (gamepad1.right_trigger > 0.3) {
                gate.setPosition(0.90);
                intake.setPower(1);
                loader.setPower(1);
            }
            else if (gamepad1.left_trigger > 0.3) {
                done = false;
                intake.setPower(-0.4);
            }
            else {
                intake.setPower(0);
                loader.setPower(0);
            }



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

            telemetry.addData("Loader Current: ", current);
            telemetry.update();
        }
    }
}
