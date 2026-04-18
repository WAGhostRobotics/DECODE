package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoUtil.ShooterPID;
import org.firstinspires.ftc.teamcode.AutoUtil.TurretPID;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class Shooter {

    Servo rightHood;
    double hoodAdjustmentConstant = 0.004;
    double hoodPos;
    Servo turret1, turret2, turret3;
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
    double P = 0.045, I=0.00, D = 0, F = 0.0045, S = 0.05;
    double currentVelocity, targetVelocity, shooterError, power;
    public static double shootSpeed = 187;
    public static double farShootSpeed = 230;
    public static double intakeShootPower = 1;
    private final int standByVelocity = 100;
    boolean ready = false;

    private ShooterPID pidController;
    private static double ninetyValue = 0.28;
    public static double backlashIncrement = 0.005;

    private static double zero = 0.495;
    double turretTargetPos;
    int shooterThreshold = 3;
    ElapsedTime shootTimer;
    ElapsedTime delay;

    double delayTime = 0.3;
    double shootTime = 0.15;

    public void init(HardwareMap hardwareMap) {
        shootTimer = new ElapsedTime();
        delay = new ElapsedTime();
        pidController = new ShooterPID(P, I, D, F, S);
        pidController.setIntegrationBounds(-10000000, 10000000);

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHood = hardwareMap.get(Servo.class, "rightHood");

        targetVelocity = 0;
    }

    public void init(HardwareMap hardwareMap, boolean teleop) {
        shootTimer = new ElapsedTime();
        delay = new ElapsedTime();
        pidController = new ShooterPID(P, I, D, F, S);
        pidController.setIntegrationBounds(-10000000, 10000000);



        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

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
        return targetVelocity != 0 && Math.abs(shooterError)<2;
    }

    public double getCurrentVelocity() {
        currentVelocity = wheel1.getVelocity(AngleUnit.RADIANS) * 48; // mm
        return currentVelocity;
    }

    public String getVelocities() {
        return "Angular (Radians): " + wheel1.getVelocity(AngleUnit.RADIANS) + "\n" +
                "Angular (Degrees): " + wheel1.getVelocity(AngleUnit.DEGREES) + "\n" +
                "Ticks: " + wheel1.getVelocity() + "\n";
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
        wheel1.setPower(-power);
        wheel2.setPower(power);
    }

    public void standBy() {
        setTargetVelocity(standByVelocity);
    }

    public void stop() {
        ready = false;
        Gus.intake.shootStop();
    }

    public void shoot() {
        if (!ready && Math.abs(shooterError) < shooterThreshold) {
            ready = true;
        }
        if (ready) {
            Gus.intake.shoot();
        }
        else {
            Gus.intake.loaderStop();
        }
    }

    public void shootSlowMotion() {
        if (!ready && Math.abs(shooterError) < shooterThreshold) {
            ready = true;
        }
        if (ready) {
            if (shootTimer.seconds() < shootTime) {
                Gus.intake.shoot();
                delay.reset();
            }
            else if (delay.seconds() < delayTime) {
                Gus.intake.shootStop();
                Gus.intake.rollerStop();
            }
            else {
                shootTimer.reset();
            }
        }
        else {
            Gus.intake.loaderStop();
        }
    }

    public void popDown() {
    }


    public void setTargetVelocity(double velocity) {
        if (velocity == 0) {
            ready = false;
        }
        velocity = Range.clip(velocity, 0, 250);
        targetVelocity = velocity;
        shooterError = targetVelocity - currentVelocity;
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
                "\nHood: " + hoodPos +
                "\nShoot Timer: " + shootTimer.seconds() +
                "\nDelay Timer: " + delay.seconds() +
                "\nReady: " + ready;
    }

    public String getTurretTelemetry() {
        return "TargetPos: " + turretTargetPos +
                "\nAngle: " + getTurretAngle();
    }

    public void setIntake(double pw){

    }

    public void setTurretTargetPos(double position) {
        if (Double.isNaN(position)) {
            return;
        }
        if (Math.abs(turretTargetPos-position) <= ninetyValue/90.0)
            return;

        position = Range.clip(position, 0, 1);
        turretTargetPos = position;
        turret1.setPosition(turretTargetPos+backlashIncrement);
        turret2.setPosition(turretTargetPos-backlashIncrement);
    }

    public void updateTurret() {
        turret1.setPosition(turretTargetPos+backlashIncrement);
        turret2.setPosition(turretTargetPos-backlashIncrement);
    }

    public void setFullPowerThreshold(double k) {
    }

    public void resetTurret() {
        setTurretTargetPos(zero);
    }

    public double getTurretAngle() {
        return (turretTargetPos-zero)*90/ninetyValue;
    }

    public void lose() {
        turret1.getController().pwmDisable();
        turret2.getController().pwmDisable();
    }

    public double getPosition() {
        return turretTargetPos;
    }

    public static double angleToPosition(double angle) {
        angle = normalizeDegrees(angle);
        return (double) (angle/90.0) * ninetyValue + zero;
    }

    public void setHood(double pos) {
        if (Double.isNaN(pos)) {
            return;
        }
        hoodPos = Range.clip(pos, 0.3, 1);
        rightHood.setPosition(1-hoodPos);
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
//        turretController.setPID(p, i, d);
    }

    public void setFineTurretPID(double p, double i, double d) {
//        fineTurretController.setPID(p, i, d);
    }

    public void setShooterThreshold(int threshold) {
        shooterThreshold = threshold;
    }

    public void setTurretKStatic(double k) {

//        turretKStatic = k;
    }

    public void setHoodAdjustmentConstant(double k) {
        hoodAdjustmentConstant = k;
    }

    public double getBallVelocity(double velocity) {
        return (velocity / 48 * 1.5);
    }

    public void setDelayTime(double k) {
        delayTime = k;
    }

    public void setShootTime(double k) {
        shootTime = k;
    }
}
