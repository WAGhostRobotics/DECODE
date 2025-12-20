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

    Servo popper, rightHood, leftHood;
    DcMotorEx turret;
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
    double P = 0.125, I=0.00275, D = 0;
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
        turretController = new PIDController(0.003, 0.0005, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        popper = hardwareMap.get(Servo.class, "popper");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetVelocity = 0;
    }

    public void init(HardwareMap hardwareMap, boolean teleop) {
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new PIDController(0.003, 0.0005, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);


        turret = hardwareMap.get(DcMotorEx.class, "turret");
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        popper = hardwareMap.get(Servo.class, "popper");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        if (!teleop)
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        wheel2.setPower(power);
    }

    public void standBy() {
        setTargetVelocity(standByVelocity);
    }

    public void stop() {
        popper.setPosition(PopperPos.RETRACT.getPosition());
        Bob.intake.rollerStop();
    }

    public void shoot() {
        popper.setPosition(PopperPos.POP.getPosition());
        if (reachedVelocity()) {
            Bob.intake.rollerIn();
        }
        else {
            Bob.intake.rollerStop();
        }
    }

    public void autoShoot() {
        popper.setPosition(PopperPos.POP.getPosition());
        Bob.intake.rollerIn();
    }
    public void popDown() {
        popper.setPosition(PopperPos.RETRACT.getPosition());
    }

    public void popUp() {
        popper.setPosition(PopperPos.POP.getPosition());
    }


    public void setTargetVelocity(double velocity) {
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
                "\nPower: " + power;
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
        position = Range.clip(position, -680, 800);
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
        turret.setPower(turretPower);
    }

    public double getTurretAngle() {
        currentPosition = turret.getCurrentPosition();        // Take this out eventually (kills loop speeds)
        return ((double)-currentPosition/434) * 90;
    }

    public static int angleToPosition(double angle) {
        return (int)((angle/90.0)*(-434));
    }

    public void setHood(double pos) {
        leftHood.setPosition(pos);
        rightHood.setPosition(1-pos);
    }

    public static double hoodAngleToPos(double angle) {
        return ((angle-27.0)/36.0)*0.7;
    }

    public void setTurretPID(double p, double i, double d) {
        turretController.setPID(p, i, d);
    }
}
