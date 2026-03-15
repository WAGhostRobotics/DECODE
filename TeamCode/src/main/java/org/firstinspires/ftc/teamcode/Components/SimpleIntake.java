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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SimpleIntake {
    private static final Logger log = LoggerFactory.getLogger(SimpleIntake.class);
    DcMotorEx intake;
    DcMotorEx loader;
    RevColorSensorV3 distance;
    RevColorSensorV3 intakeDistance;
    RevColorSensorV3 intakeDistanceTwo;

    RevColorSensorV3 midDistance;
    // Loop optimizer
    long lastTime = System.nanoTime();
    double highSensorDistance = 0;
    double lowerSensorDistance = 0;
    double secondLowerSensorDistance = 0;
    double midSensorDistance = 0;

    double currentLoader;
    double currentIntake;
    Servo gate;
    public boolean gateOpen = true;
    double power;
    double outPower;
    public static final double oneBallInThreshold = 2.0;
    public static double midSensorThreshold = 2.7;
    public static double rampFullThreshold = 7.5;
    public static double secondFullThreshold = 3.2;
    boolean initialized;

    private double[] rampReadings;
    private double[] midReadings;
    double minReading, maxReading, avgReading;
    double avgReadingMid;
    private final int numReadings = 5;
    private int index = 0;
    public static final double currentThresholdLoader = 5.0;
    public static final double currentThresholdIntake = 4.0;
    boolean oneBallIn, twoBallIn;
    boolean full;
    ElapsedTime loaderTimer, intakeTimer;
    double timerThreshold = 0.3;
    double intakeTimerThreshold = 0.35;
    double antiShootPower = -0.08;
    HardwareMap hwMap;


    public SimpleIntake(HardwareMap hardwareMap) {
        hwMap = hardwareMap;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        loader = hardwareMap.get(DcMotorEx.class, "loader");
        distance = hardwareMap.get(RevColorSensorV3.class, "distance");
        intakeDistance = hardwareMap.get(RevColorSensorV3.class, "intakeDistance");
        intakeDistanceTwo = hardwareMap.get(RevColorSensorV3.class, "intakeDistanceTwo");
        midDistance = hardwareMap.get(RevColorSensorV3.class, "midDistance");
        gate = hardwareMap.get(Servo.class, "gate");
        full = false;
        closeGate();
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 1;
        outPower = 0.6;
        rampReadings = new double[numReadings];
        midReadings = new double[numReadings];
        index = 0;
        loaderTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        initialized = false;
    }

    public void updateIntake() {
        long now = System.nanoTime();
        if (now - lastTime > 33_000_000) {
            lowerSensorDistance = intakeDistance.getDistance(DistanceUnit.CM);
            secondLowerSensorDistance = intakeDistanceTwo.getDistance(DistanceUnit.CM);
            midSensorDistance = midDistance.getDistance(DistanceUnit.CM);
            if (lowerSensorDistance > 15) {
                intakeDistance = hwMap.get(RevColorSensorV3.class, "intakeDistance");
            }
            index = (index + 1) % numReadings;
            rampReadings[index] = lowerSensorDistance;
            midReadings[index] = midSensorDistance;
            lastTime = now;
            if (!oneBallIn)
                highSensorDistance = distance.getDistance(DistanceUnit.CM);
            else
                highSensorDistance = 10;
            getMaxAndMin();
            getCurrentDrawLoader();
        }

        if (!oneBallIn && (highSensorDistance <= oneBallInThreshold || currentLoader > currentThresholdLoader) ) {
            oneBallIn = true;
            loaderStop();
        }

        if (oneBallIn && (avgReadingMid <= midSensorThreshold)) {
            twoBallIn = true;
        }
        if (!twoBallIn) {
            full = false;
        }

        if (twoBallIn && ((maxReading <= rampFullThreshold) || (secondLowerSensorDistance<=secondFullThreshold))) {
            if (intakeTimer.seconds() > intakeTimerThreshold) {
                full = true;
                power = 0;
                Gus.ledLights.green();
            }
        }
        else if (minReading >= rampFullThreshold) {
            power = 1;
            Gus.ledLights.orange();
            intakeTimer.reset();
        }
        else {
            intakeTimer.reset();
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
            if (twoBallIn) {
                loader.setPower(antiShootPower);
            }
            else {
                loader.setPower(0);
            }
        }
    }

    public void bruteRollerIn() {
        full = false;
        intake.setPower(1);
    }

    public void setRampFullThreshold() {
        initialized = true;
        rampFullThreshold = avgReading-0.65;
        midSensorThreshold = avgReadingMid-1.3;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void slowRollerIn() {
        intake.setPower(0.5);
    }
    public void rollerOut() {
        oneBallIn = false;
        intake.setPower(-outPower);
    }

    public void setFull() {
        oneBallIn = true;
        twoBallIn = true;
        full = true;
        power = 0;
    }

    public void setBallIn(boolean ballIn) {
        oneBallIn = ballIn;
        if (!ballIn) {
            loaderTimer.reset();
            power = 1;
            full = false;
            twoBallIn = false;
            Gus.ledLights.orange();
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

        return "Done: " + oneBallIn +
                "\nTwoBall In: " + twoBallIn +
                "\nFull: " + full +
                "\nHigh Distance: " + highSensorDistance +
                "\nRamp Distance: " + lowerSensorDistance +
                "\nRamp 2 Distance: " + secondLowerSensorDistance +
                "\nMid Distance: " + midSensorDistance +
                "\nMid Threshold: " + midSensorThreshold +
                "\nLow Threshold: " + rampFullThreshold;
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
    public boolean isTwoBallIn() {
        return twoBallIn;
    }
    public boolean isFull() {
        return full;
    }

    public void getMaxAndMin() {
        double sum = 0;
        double midSum = 0;
        double max = rampReadings[0];
        double min = rampReadings[0];
        for (int i = 0; i<numReadings; i++) {
            double reading = rampReadings[i];
            double midReading = midReadings[i];
            if (reading > max)
                max = reading;
            if (reading < min)
                min = reading;
            sum += reading;
            midSum += midReading;

        }
        avgReading = sum/numReadings;
        maxReading = max;
        minReading = min;
        avgReadingMid = midSum/numReadings;
    }

    public void setAntiShootPower(double pw) {
        antiShootPower = pw;
    }

}
