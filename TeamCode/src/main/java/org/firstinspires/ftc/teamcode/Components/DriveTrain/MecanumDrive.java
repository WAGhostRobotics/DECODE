package org.firstinspires.ftc.teamcode.Components.DriveTrain;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.Components.RI3W.Constants;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Config
public class MecanumDrive implements Drivetrain {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private double sin;
    private double cos;
    private double maxMovement;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public static double strafeMultiplier = 1;
    private final double velocityThreshold = 5;

    public final double voltageConstant = 12.3;
    public static double Kv = 0.0142;
    private double KStaticTurn = 0.18;
    private double headingErrorThreshold = 2;
    private final PIDController translationalControlX = new PIDController(Constants.translationalXP,Constants.translationalXI, Constants.translationalXD);
    private final PIDController translationalControlY =  new PIDController(Constants.translationalYP, Constants.translationalYI, Constants.translationalYD);
    private final PIDController headingControl = new PIDController(Constants.headingP, Constants.headingI, Constants.headingD);
    private double kStaticX = Constants.kStaticX;                 // Minimum power before robot moves in X direction
    private double kStaticY = Constants.kStaticY;
    private double kStaticTurn = Constants.kStaticTurn;
    String telemetry = "";


    public MecanumDrive(HardwareMap hardwareMap){
        translationalControlX.setIntegrationBounds(-10000000, 10000000);
        translationalControlY.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        backRight = hardwareMap.get(DcMotorEx.class, "rb");

//        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower){

        driveCommon(magnitude, theta, driveTurn);

        //scales
        frontLeftPower /= magnitude + Math.abs(driveTurn);
        frontRightPower /= magnitude + Math.abs(driveTurn);
        backLeftPower /= magnitude + Math.abs(driveTurn);
        backRightPower /= magnitude + Math.abs(driveTurn);


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

        // 1 -1 1 -1
        // -1 -1 1 1

        // 1 -1 -1 1

    }



    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower){

        driveCommon(magnitude, theta, driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
        }


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);

        telemetry = "FrontLeft: " + frontLeftPower + " \nFrontRight: " + frontRightPower + " \nBackLeft: " + backLeftPower + " \nBackRight" + backRightPower;
    }

    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage){

        driveCommon(magnitude, theta, driveTurn);

        //scales
        frontLeftPower /= magnitude + Math.abs(driveTurn);
        frontRightPower /= magnitude + Math.abs(driveTurn);
        backLeftPower /= magnitude + Math.abs(driveTurn);
        backRightPower /= magnitude + Math.abs(driveTurn);


        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

    }


    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage){

        driveCommon(magnitude, theta, driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
        }


        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }


    @Override
    public String getTelemetry(){
        return telemetry;
    }

    public void scaleByVoltage(double voltage){
        frontLeftPower /= voltage;
        frontRightPower /= voltage;
        backLeftPower /= voltage;
        backRightPower /= voltage;

        frontLeftPower *= voltageConstant;
        frontRightPower *= voltageConstant;
        backLeftPower *= voltageConstant;
        backRightPower *= voltageConstant;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

    }

    public void driveCommon(double magnitude, double theta, double driveTurn){
        theta += 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta)) * strafeMultiplier;
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = (magnitude * cos / maxMovement - driveTurn);
        frontRightPower = (magnitude * sin / maxMovement + driveTurn);
        backLeftPower = (magnitude * sin / maxMovement - driveTurn);
        backRightPower = (magnitude * cos / maxMovement + driveTurn);
        telemetry = "FrontLeft: " + frontLeftPower + " \nFrontRight: " + frontRightPower + " \nBackLeft: " + backLeftPower + " \nBackRight" + backRightPower;


    }

    public double totalCurrent() {
        return frontLeft.getCurrent(CurrentUnit.AMPS) + frontRight.getCurrent(CurrentUnit.AMPS) +
                backLeft.getCurrent(CurrentUnit.AMPS) + backRight.getCurrent(CurrentUnit.AMPS);
    }

    public void holdChassis(Pose2D currentPose, Pose2D targetPose, double driveTurn) {
        double targetX = targetPose.getX(DistanceUnit.INCH);
        double targetY = targetPose.getY(DistanceUnit.INCH);
        double xError = targetX - currentPose.getX(DistanceUnit.INCH);
        double yError = targetY - currentPose.getY(DistanceUnit.INCH);
        double currentHeading = currentPose.getHeading(DEGREES);

        double translationalError = Math.hypot(xError, yError);
        double theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);       // Theta relative to robot
        xError = Math.cos(Math.toRadians(theta))*translationalError;                // X and Y relative to robot
        yError = Math.sin(Math.toRadians(theta))*translationalError;
        double xPower = translationalControlX.calculate(0, xError);
        double yPower = translationalControlY.calculate(0, yError);
        xPower = xPower + Math.signum(xPower)* kStaticX;
        yPower = yPower + Math.signum(yPower)* kStaticY;
        if (Math.abs(xError) < 0.6) {
            xPower = 0;
            translationalControlX.reset();
        }

        if (Math.abs(yError) < 0.6) {
            yPower = 0;
            translationalControlY.reset();
        }

        double magnitude = Math.hypot(xPower, yPower);
        drive(magnitude, theta, driveTurn, Constants.movementPower);

    }
}