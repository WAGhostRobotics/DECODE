package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.Core.Gus;

@Config
@TeleOp
public class Chelmsford extends OpMode {
    public static boolean lf_R = false;
    public static boolean rf_R = false;
    public static boolean lb_R = false;
    public static boolean rb_R = false;
    DcMotorEx lf, rf, lb, rb;

    String data;
    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);


        if (lf_R) {
            lf.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (rf_R) {
            rf.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (lb_R) {
            lb.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (rb_R) {
            rb.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void loop() {
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;



        double driveTurn = -gamepad1.right_stick_x;

        double magnitude = Math.hypot(x, y);
        double theta = Math.toDegrees(Math.atan2(y, x));
        drive(magnitude, theta, driveTurn, 0.9);

        telemetry.addData("Data: ", data);
        telemetry.update();


    }

    public void drive(double magnitude, double theta, double driveTurn, double movementPower){

        driveCommon(magnitude, theta, driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
        }


        lf.setPower(movementPower*frontLeftPower);
        rf.setPower(movementPower*frontRightPower);
        lb.setPower(movementPower*backLeftPower);
        rb.setPower(movementPower*backRightPower);

        data = "FrontLeft: " + frontLeftPower + " \nFrontRight: " + frontRightPower + " \nBackLeft: " + backLeftPower + " \nBackRight" + backRightPower;
    }

    public void driveCommon(double magnitude, double theta, double driveTurn){
        theta += 45;

        //sin and cos of robot movement
        double sin = Math.sin(Math.toRadians(theta));
        double cos = Math.cos(Math.toRadians(theta));
        double maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = (magnitude * cos / maxMovement - driveTurn);
        frontRightPower = (magnitude * sin / maxMovement + driveTurn);
        backLeftPower = (magnitude * sin / maxMovement - driveTurn);
        backRightPower = (magnitude * cos / maxMovement + driveTurn);
        data = "FrontLeft: " + frontLeftPower + " \nFrontRight: " + frontRightPower + " \nBackLeft: " + backLeftPower + " \nBackRight" + backRightPower;


    }

}
