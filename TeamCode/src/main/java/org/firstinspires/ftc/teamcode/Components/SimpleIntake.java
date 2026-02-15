package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Gus;

public class SimpleIntake {
    DcMotorEx intake;
    DcMotorEx loader;
    RevColorSensorV3 distance;
    RevColorSensorV3 intakeDistance;
    // Loop optimizer
    long lastTime = System.nanoTime();
    double highSensorDistance = 0;
    double lowerSensorDistance = 0;

    double currentLoader;
    double currentIntake;
    Servo gate;
    public boolean gateOpen = true;
    double power;
    double outPower;
    public static final double oneBallInThreshold = 2.0;
    public static double rampFullThreshold = 7.5;
    private double[] rampReadings;
    double minReading, maxReading, avgReading;
    private final int numReadings = 5;
    private int index = 0;
    public static final double currentThresholdLoader = 5.0;
    public static final double currentThresholdIntake = 4.0;
    boolean oneBallIn;
    boolean full;
    ElapsedTime loaderTimer;
    double timerThreshold = 0.5;
    double antiShootPower = -0.09;


    public SimpleIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        loader = hardwareMap.get(DcMotorEx.class, "loader");
        distance = hardwareMap.get(RevColorSensorV3.class, "distance");
        intakeDistance = hardwareMap.get(RevColorSensorV3.class, "intakeDistance");
        gate = hardwareMap.get(Servo.class, "gate");
        full = false;
        closeGate();
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 1;
        outPower = 0.6;
        rampReadings = new double[numReadings];
        index = 0;
        loaderTimer = new ElapsedTime();
    }

    public void updateIntake() {
        long now = System.nanoTime();
        if (now - lastTime > 16_500_000) {
            lowerSensorDistance = intakeDistance.getDistance(DistanceUnit.CM);
            index = (index + 1) % numReadings;
            rampReadings[index] = lowerSensorDistance;
            lastTime = now;
            highSensorDistance = distance.getDistance(DistanceUnit.CM);
            getMaxAndMin();
            getCurrentDrawLoader();
            getCurrentDrawIntake();
        }

        if (!oneBallIn && (highSensorDistance <= oneBallInThreshold || currentLoader > currentThresholdLoader) ) {
            oneBallIn = true;
            loaderStop();
        }

        if (oneBallIn && (maxReading <= rampFullThreshold)) {
            full = true;
            power = 0;
        }
        else if (minReading >= rampFullThreshold) {
            power = 1;
        }
    }

    public void rollerIn() {
        if (!oneBallIn && !gateOpen) {
            intake.setPower(power);
            if (loaderTimer.seconds() > timerThreshold) {
                loader.setPower(1);
            }
            else {
                loader.setPower(0);
            }
        }
        else {
            intake.setPower(power);
            loader.setPower(antiShootPower);
        }
    }

    public void bruteRollerIn() {
        full = false;
        intake.setPower(1);
    }

    public void setRampFullThreshold() {
        rampFullThreshold = avgReading-1;
    }

    public void slowRollerIn() {
        intake.setPower(0.5);
    }
    public void rollerOut() {
        oneBallIn = false;
        intake.setPower(-outPower);
    }

    public void setBallIn(boolean ballIn) {
        oneBallIn = ballIn;
        if (!ballIn) {
            loaderTimer.reset();
            power = 1;
            full = false;
        }
    }

    public void rollerStop() {
        intake.setPower(0);
        loaderStop();
    }

    public void shoot() {
        intake.setPower(1);
        loader.setPower(1);
    }
    public void shootStop() {
        loader.setPower(0);
    }

    public void loaderStop() {
        loader.setPower(0);
    }

    public void setPower(double pw) {
        power = pw;
    }

    public void openGate() {
        gate.setPosition(0);
        gateOpen = true;
        oneBallIn = true;
    }

    public void closeGate() {
        gate.setPosition(0.76);
        gateOpen = false;
    }

    public String getTelemetry() {

        return "Power: " + power +
                "\nCurrent: " + currentLoader +
                "\nDone: " + oneBallIn +
                "\nRamp Distance: " + lowerSensorDistance +
                "\nHigh Distance: " + highSensorDistance +
                "\nThreshold: " + rampFullThreshold;
    }

    public double getCurrentDrawLoader() {
        currentLoader = loader.getCurrent(CurrentUnit.AMPS);
        return currentLoader;
    }

    public double getCurrentDrawIntake() {
        currentIntake = intake.getCurrent(CurrentUnit.AMPS);
        return currentIntake;
    }

    public void updateGate() {
        if (Gus.shooter.reachedVelocity()) {
            openGate();
        }
    }

    public boolean isOneBallIn() {
        return oneBallIn;
    }

    public boolean isFull() {
        return full;
    }

    public void getMaxAndMin() {
        double sum = 0;
        double max = rampReadings[0];
        double min = rampReadings[0];
        for (double reading: rampReadings) {
            if (reading > max)
                max = reading;
            if (reading < min)
                min = reading;
            sum += reading;
        }
        avgReading = sum/numReadings;
        maxReading = max;
        minReading = min;
    }

    public void setAntiShootPower(double pw) {
        antiShootPower = pw;
    }

}
