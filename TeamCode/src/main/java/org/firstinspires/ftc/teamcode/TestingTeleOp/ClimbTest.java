package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp

public class ClimbTest extends LinearOpMode {
    public static int targetPos = 0;
    public static double testPower = 0;
    public static double P = 0, I = 0, D = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        PIDController climbControl = new PIDController(P, I, D);
        climbControl.setIntegrationBounds(-1000000, 1000000);
        DcMotorEx climber = hardwareMap.get(DcMotorEx.class, "climb");
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setDirection(DcMotorSimple.Direction.REVERSE);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            climbControl.setPID(P, I, D);
            int currentPos = climber.getCurrentPosition();
            int error = targetPos - currentPos;
            double power = Range.clip(climbControl.calculate(0, error), -1, 1);
            climber.setPower(power);
//            if (gamepad1.a) {
//                climber.setPower(testPower);
//            }
//            else if (gamepad1.b) {
//                climber.setPower(-testPower);
//            }
//            else {
//                climber.setPower(0);
//            }
            telemetry.addData("Power: ", power);
            telemetry.addData("CurrentPos: ", currentPos);
            telemetry.addData("TargetPos: ", targetPos);
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
    }
}
