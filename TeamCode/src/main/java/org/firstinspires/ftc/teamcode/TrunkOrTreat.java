package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// all calculations done in cm and degrees :D
//@Config
//@TeleOp(name = "Trunk or Treat")
public class TrunkOrTreat {
    DcMotorEx base, first, second;
    Servo wrist;
    CRServo claw;
    PIDController controller;
    private double P, I, D;

    int targetPos;

    double L1 = 42.5, L2 = 38.73, L3;
    double lambda, alpha, theta1, theta2;
    double j1ErrorDegrees, j2ErrorDegrees, j1ErrorTicks, j2ErrorTicks;
    double j1TargetPosition, j2TargetPosition;
    double power1, power2;

    // P = 0.05 for arm1



//    @Override
//    public void runOpMode() throws InterruptedException {
    public TrunkOrTreat(HardwareMap hardwareMap){
        base = hardwareMap.get(DcMotorEx.class, "base");
        first = hardwareMap.get(DcMotorEx.class, "first");
        second = hardwareMap.get(DcMotorEx.class, "second");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(CRServo.class, "claw");

        wrist.setPosition(0);

        controller = new PIDController(P, I, D);
        controller.setIntegrationBounds(-1000000, 1000000);

        base.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        first.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        second.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        first.setDirection(DcMotorEx.Direction.REVERSE);
        second.setDirection(DcMotorEx.Direction.REVERSE);

//        first.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        second.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        base.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        first.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        second.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


//        waitForStart();
//        while (opModeIsActive()) {
//
//            if (gamepad1.left_stick_y != 0){
//                if (first.getCurrentPosition() > 1569 || first.getCurrentPosition() < 0){
//                    first.setPower(0);
//                } else{
//                    first.setPower(gamepad1.left_stick_y*(-0.5));
//                }
//            }
//            if (gamepad1.right_stick_y != 0) {
//                if (second.getCurrentPosition() > 488 || second.getCurrentPosition() < 0){
//                    second.setPower(0);
//                } else{
//                    second.setPower(gamepad1.right_stick_y*(-0.5));
//                }
//            }
//            if (gamepad1.right_trigger > 0.05){
//                base.setPower(gamepad1.right_trigger*0.5);
//            }
//            if (gamepad1.left_trigger > 0.05) {
//                base.setPower(-gamepad1.left_trigger*0.5);
//            }


//            setPID(p, i, d);
//            goToPosition(X,Y);
//            update();

//
//            double arm2Pos = second.getCurrentPosition()+ first.getCurrentPosition();
//
////
//            telemetry.addData("rotation pos: ", base.getCurrentPosition());
//            telemetry.addLine("J1 target position is " + j1TargetPosition);
//            telemetry.addData("J1 current pos: ", first.getCurrentPosition());
//            telemetry.addLine("J2 target position is " + j2TargetPosition);
//            telemetry.addData("J2 current pos: ", second.getCurrentPosition());
//            telemetry.addData("wrist pos: ", wrist.getPosition());
//            telemetry.addData("real arm 2 position: ", arm2Pos);
//            telemetry.update();
        }

    public void goToPosition (double x, double y) {
        L3 = Math.hypot(x, y); // hypotenuse
        double maxReach = L1 + L2 - 0.1; // small margin
        if (L3 > maxReach) L3 = maxReach;
        if (L3 < Math.abs(L1 - L2)) L3 = Math.abs(L1 - L2);

        lambda = Math.atan2(y, x);
        alpha = Math.acos(((L1 * L1) + (L3 * L3) - (L2 * L2)) / (2 * L1 * L3));
        theta1 = Math.toDegrees(lambda + alpha);
        theta2 = Math.toDegrees(Math.acos(((L1 * L1) + (L2 * L2) - (L3 * L3)) / (2 * L1 * L2)));


        j1ErrorDegrees = theta1 - Math.toDegrees(first.getCurrentPosition());
        j2ErrorDegrees = theta2 - Math.toDegrees(second.getCurrentPosition());
        j1ErrorTicks = j1ErrorDegrees * (.0824675);
        j2ErrorTicks = j2ErrorDegrees * .783582;

        j1TargetPosition = first.getCurrentPosition() + j1ErrorTicks;
        j2TargetPosition = second.getCurrentPosition() + j2ErrorTicks;
    }
    public void setPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        controller.setPID(P, I, D);
    }

    public void setTargetPosition(int targetPos){
         this.targetPos = targetPos;
    }

    public void update() {
//        double currentPos1 = first.getCurrentPosition();
        double currentPos2 = second.getCurrentPosition();
//        double error1 = targetPos - currentPos1;
        double error2 = targetPos - currentPos2;
//        double power1 = controller.calculate(0, error1);
        double power2 = controller.calculate(0, error2);

//        power1 = Math.max(-1, Math.min(1, power1));
        power2 = Math.max(-1, Math.min(1, power2));

//        first.setPower(power1*0.5);
        second.setPower(power2*0.5);
    }
}