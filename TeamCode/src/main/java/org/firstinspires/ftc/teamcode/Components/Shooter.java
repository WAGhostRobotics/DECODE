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
    CRServo s1;
    CRServo s2;
    DcMotorEx encoder;
    public enum PopperPos {
        POP(0.7075), RETRACT(0.82);
        private final double pos;
        PopperPos(double val) {this.pos = val;}
        public double getPosition() {
            return pos;
        }
    }
    DcMotorEx wheel1;
    DcMotorEx wheel2;
    double P = 0.05, I=0.0035, D = 0;
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
        s1 = hardwareMap.get(CRServo.class, "turr1");
        s2 = hardwareMap.get(CRServo.class, "turr2");
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new PIDController(0.000175, 0.0000001, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);


        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        popper = hardwareMap.get(Servo.class, "popper");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = hardwareMap.get(DcMotorEx.class, "rb");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        targetVelocity = 0;
    }

    public void init(HardwareMap hardwareMap, boolean teleop) {
        s1 = hardwareMap.get(CRServo.class, "turr1");
        s2 = hardwareMap.get(CRServo.class, "turr2");
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        pidController = new PIDController(P, I, D);
        pidController.setIntegrationBounds(-10000000, 10000000);
        turretController = new PIDController(0.0001, 0.00007, 0);
        turretController.setIntegrationBounds(-10000000, 10000000);


        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        popper = hardwareMap.get(Servo.class, "popper");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = hardwareMap.get(DcMotorEx.class, "rb");
        if (!teleop)
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
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
        Bob.intake.stopSpindexer();
    }

    public void shoot() {
        if (reachedVelocity()) {
            popper.setPosition(PopperPos.POP.getPosition());
            Bob.intake.updateSpindexer();
        }
        else {
            popper.setPosition(PopperPos.RETRACT.getPosition());
        }
    }

    public int realShoot() {
        if (!Bob.intake.isFinished()) {
            popper.setPosition(PopperPos.RETRACT.getPosition());
        }
        else {
            popper.setPosition(PopperPos.POP.getPosition());
        }
        int ballsShot = Bob.intake.spindexerShoot();
        Bob.intake.rollerIn();
        return ballsShot;

    }

    public void rapidShoot() {
        popper.setPosition(PopperPos.POP.getPosition());
        Bob.intake.rollerIn();
        Bob.intake.rotateCW();
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
        turretTargetPos = position;
    }

    public void updateTurret() {
        turretError = turretTargetPos - currentPosition;
        if (Math.abs(turretError)<150) {
            turretPower = 0;
            turretController.reset();
            return;
        }
        turretPower = turretController.calculate(0, turretError);

        turretPower = Range.clip(turretPower, -1, 1);

        s1.setPower(turretPower);
        s2.setPower(-turretPower);
    }

    public double getTurretAngle() {
        currentPosition = -encoder.getCurrentPosition();        // Take this out eventually (kills loop speeds)
        return ((double)-currentPosition/8301.0) * 90;
    }

    public static int angleToPosition(double angle) {
        return (int)((angle/90.0)*(-8200.0));
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
