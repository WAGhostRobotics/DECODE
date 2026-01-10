package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.Bob;

public class Shooter {

    Servo rightHood;
    double hoodPos;
    CRServo turret1, turret2;
    public enum PopperPos {
        POP(0.596), RETRACT(0.6439);
        private final double pos;
        PopperPos(double val) {this.pos = val;}
        public double getPosition() {
            return pos;
        }
    }
    DcMotorEx wheel1;
    DcMotorEx wheel2;
    double P = 0.07, I=0.004, D = 0;
    double currentVelocity, targetVelocity, error, power;
    public static double shootSpeed = 187;
    public static double farShootSpeed = 230;
    public static double intakeShootPower = 1;
    private final int standByVelocity = 100;

    private PIDController pidController;
    private PIDController turretController;
    int turretTargetPos, currentPosition, turretError;
    double turretPower;

    public void init(HardwareMap hardwareMap) {
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new PIDController(0.00005, 0.000005, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);

        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");

        targetVelocity = 0;
    }

    public void init(HardwareMap hardwareMap, boolean teleop) {
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new PIDController(0.00005, 0.000005, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);


        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        if (!teleop) {
            wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        targetVelocity = 0;
    }

    public boolean reachedVelocity() {
        return Math.abs(error)<6;
    }

    public double getCurrentVelocity() {
        currentVelocity = wheel1.getVelocity(AngleUnit.RADIANS) * 48; // mm
        return currentVelocity;
    }

    public void updateShooter() {
        if (targetVelocity == 0) {
            pidController.reset();
            wheel1.setPower(0);
            wheel2.setPower(0);
            return;
        }
        getCurrentVelocity();
        error = targetVelocity - currentVelocity;
        power = pidController.calculate(0, error);
        power = Range.clip(power, -1, 1);
        wheel1.setPower(power);
        wheel2.setPower(-power);
    }

    public void standBy() {
        setTargetVelocity(standByVelocity);
    }

    public void stop() {
        Bob.intake.shootStop();
    }

    public void shoot() {
        if (Math.abs(error) < 3) {
            Bob.intake.shoot();
        }
        else {
            Bob.intake.loaderStop();
        }
    }

    public void autoShoot() {
        Bob.intake.shoot();
    }
    public void popDown() {
    }

    public void popUp() {

    }


    public void setTargetVelocity(double velocity) {
        velocity = Range.clip(velocity, 0, 250);
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }

    public void resetPID() {
        pidController.reset();
    }

    public String getTelemetry() {
        return "Target V: " + targetVelocity +
                "\nCurrent V: " + currentVelocity +
                "\nShooter Error: " + error +
                "\nPower: " + power +
                "\nHood: " + hoodPos;
    }

    public String getTurretTelemetry() {
        return "TargetPos: " + turretTargetPos +
                "\nCurrent Pos: " + currentPosition +
                "\nAngle: " + getTurretAngle() +
                "\nError: " + turretError +
                "\nTurret Power: " + turretPower;
    }

    public void setIntake(double pw){

    }

    public void setTurretTargetPos(int position) {
        position = Range.clip(position, -27000, 24500);
        turretTargetPos = position;
    }

    public void updateTurret() {
        turretError = turretTargetPos - currentPosition;
        if (Math.abs(turretError)<10) {
            turretPower = 0;
            turretController.reset();
            return;
        }
        turretPower = turretController.calculate(0, turretError);
        turretPower = Range.clip(turretPower, -1, 1);
        turret1.setPower(turretPower);
        turret2.setPower(turretPower);
    }

    public double getTurretAngle() {
        currentPosition = wheel2.getCurrentPosition();        // Take this out eventually (kills loop speeds)
        return ((double)-currentPosition/17000) * 90;
    }

    public double getPosition() {
        return wheel2.getCurrentPosition();
    }

    public static int angleToPosition(double angle) {
        return (int)((angle/90.0)*(-17000));
    }

    public void setHood(double pos) {
        hoodPos = Range.clip(pos, 0, 1);
        rightHood.setPosition(1-pos);
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public static double hoodAngleToPos(double angle) {
        return ((angle-27.0)/36.0)*0.7;
    }

    public void setTurretPID(double p, double i, double d) {
        turretController.setPID(p, i, d);
    }
}
