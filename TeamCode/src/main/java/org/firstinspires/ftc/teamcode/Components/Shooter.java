package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoUtil.ShooterPID;
import org.firstinspires.ftc.teamcode.AutoUtil.TurretPID;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class Shooter {

    Servo rightHood;
    double hoodAdjustmentConstant = 0.005;
    double hoodPos;
    CRServo turret1, turret2, turret3;
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
    double P = 0.0275, I=0.00, D = 0, F = 0.00328, S = 0.09;
    double currentVelocity, targetVelocity, shooterError, power;
    public static double shootSpeed = 187;
    public static double farShootSpeed = 230;
    public static double intakeShootPower = 1;
    private final int standByVelocity = 100;
    boolean ready = false;

    private ShooterPID pidController;
    private TurretPID turretController;
    private PIDController fineTurretController;
    public static double tP = 0.000055, tI = 0.000002, tD = 0;
    public static double fTP = 0.000055, fTI = 0.00000, fTD = 0;
    private double turretKStatic = 0.00;
    int turretTargetPos, currentPosition, turretError;
    int shooterThreshold = 3;
    double turretPower;

    public void init(HardwareMap hardwareMap) {
        pidController = new ShooterPID(P, I, D, F, S);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new TurretPID(tP, tI, tD);
        turretController.setIntegrationBounds(-500000, 500000);
        fineTurretController = new PIDController(fTP, fTI, fTD);
        fineTurretController.setIntegrationBounds(-10000000, 10000000);

        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turret3 = hardwareMap.get(CRServo.class, "turret3");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");

        targetVelocity = 0;
    }

    public void init(HardwareMap hardwareMap, boolean teleop) {
        pidController = new ShooterPID(P, I, D, F, S);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new TurretPID(tP, tI, tD);
        turretController.setIntegrationBounds(-500000, 500000);
        fineTurretController = new PIDController(fTP, fTI, fTD);
        fineTurretController.setIntegrationBounds(-10000000, 10000000);


        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turret3 = hardwareMap.get(CRServo.class, "turret3");

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
        return targetVelocity != 0 && Math.abs(shooterError)<6;
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
        shooterError = targetVelocity - currentVelocity;
        power = pidController.calculate(currentVelocity, targetVelocity);
        power = Range.clip(power, -1, 1);
        wheel1.setPower(power);
        wheel2.setPower(-power);
    }

    public void standBy() {
        setTargetVelocity(standByVelocity);
    }

    public void stop() {
        ready = false;
        Gus.intake.shootStop();
    }

    public void shoot() {
        if (Math.abs(shooterError) < shooterThreshold) {
            ready = true;
        }
        if (ready) {
            Gus.intake.shoot();
        }
        else {
            Gus.intake.loaderStop();
        }
    }

    public void autoShoot() {
        Gus.intake.shoot();
    }
    public void popDown() {
    }

    public void popUp() {

    }


    public void setTargetVelocity(double velocity) {
        if (velocity == 0) {
            ready = false;
        }
        velocity = Range.clip(velocity, 0, 250);
        targetVelocity = velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setPID(double p, double i, double d, double f, double s) {
        pidController.setPIDFS(p, i, d, f, s);
    }

    public void resetPID() {
        pidController.reset();
    }

    public String getTelemetry() {
        return "Target V: " + targetVelocity +
                "\nCurrent V: " + currentVelocity +
                "\nShooter Error: " + shooterError +
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
        position = Range.clip(position, -29000, 27000);
        turretTargetPos = position;
    }

    public void updateTurret() {
        currentPosition = wheel2.getCurrentPosition();
        turretError = turretTargetPos - currentPosition;


        if (Math.abs(turretError)<125) {
            turretPower = 0;
            turret1.setPower(turretPower);
            turret2.setPower(turretPower);
            turret3.setPower(turretPower);
            return;
        }
        else if (Math.abs(turretError) < 1000) {
            turretPower = fineTurretController.calculate(0, turretError);
        }
        else {
            turretPower = turretController.calculate(0, turretError);
        }
        turretPower = turretPower + Math.signum(turretPower)*turretKStatic;
        turretPower = Range.clip(turretPower, -1, 1);
        turret1.setPower(turretPower);
        turret2.setPower(turretPower);
        turret3.setPower(turretPower);
    }

    public void setFullPowerThreshold(double k) {
        turretController.setFullPowerThreshold(k);
    }

    public void resetTurret() {
        setTurretTargetPos(0);
        turretController.reset();
        fineTurretController.reset();
    }

    public double getTurretAngle() {
        return ((double)-currentPosition/17000) * 90;
    }

    public double getPosition() {
        return wheel2.getCurrentPosition();
    }

    public static int angleToPosition(double angle) {
        return (int)((angle/90.0)*(-17000));
    }

    public void setHood(double pos) {
        if (Double.isNaN(pos)) {
            return;
        }
        hoodPos = Range.clip(pos, 0, 1);
        rightHood.setPosition(1-pos);
    }

    public void setHood(double pos, boolean adjusting) {
        if (adjusting) {
            pos = pos + shooterError*hoodAdjustmentConstant;
        }
        setHood(pos);
    }

    public double getHoodPos() {
        return hoodPos;
    }


    public void setTurretPID(double p, double i, double d) {
        turretController.setPID(p, i, d);
    }

    public void setFineTurretPID(double p, double i, double d) {
        fineTurretController.setPID(p, i, d);
    }

    public void setShooterThreshold(int threshold) {
        shooterThreshold = threshold;
    }

    public void setTurretKStatic(double k) {
        turretKStatic = k;
    }

    public void setHoodAdjustmentConstant(double k) {
        hoodAdjustmentConstant = k;
    }
}
