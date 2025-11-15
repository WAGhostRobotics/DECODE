package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.E;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.G;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.P;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class Intake {
    CRServo s1, s2;
    DcMotorEx intake;
    RevColorSensorV3 colorSensor;
    public static int slotIncrement = 2730;
    double spinPower = 0.19;
    double intakePower = 1;
    int index;
    SlotState currentState = E;
    int targetPosition = 0;
    int ballsEaten = 0;
    int ballsShot = 0;
    int error = 0;
    double power = 0;
    boolean toUpdate = true;

    PIDController spinController = new PIDController(0.00016, 0.000002, 0);
    public enum SlotState {
        P,
        G,
        E
    }
    SlotState[] slots = {G, P, P};

    SlotState[] motif;

    public Intake(HardwareMap hwMap) {
        spinController.setIntegrationBounds(-1000000, 1000000);
        index = 0;
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1 = hwMap.get(CRServo.class, "s1");
        s2 = hwMap.get(CRServo.class, "s2");
        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public Intake(HardwareMap hwMap, boolean teleOp) {
        spinController.setIntegrationBounds(-1000000, 1000000);
        index = 0;
        intake = hwMap.get(DcMotorEx.class, "intake");
        if (!teleOp)
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1 = hwMap.get(CRServo.class, "s1");
        s2 = hwMap.get(CRServo.class, "s2");
        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public void rollerIn() {
//        if (colorSensor.getDistance(DistanceUnit.MM)<=20) {
//            if (currentState == PURPLE) {
//                // delay
//                index = (index+1)%3;
//
//            }
//        }
        intake.setPower(intakePower);
    }

    public void rollerOut() {
        intake.setPower(-intakePower);
    }

    public void rollerStop() {
        intake.setPower(0);
    }
    public void rotateCW() {
        toUpdate = false;
        s1.setPower(spinPower);
        s2.setPower(spinPower);
    }

    public void rotateCCW() {
        toUpdate = false;
        s1.setPower(-spinPower);
        s2.setPower(-spinPower);
    }

    public void stopSpindexer() {
        s1.setPower(0);
        s2.setPower(0);
    }

    public void setTargetPosition(int pos) {

        targetPosition = pos;
    }
    public void updateSpindexer() {
        if (!toUpdate)
            return;
        int currentPosition = intake.getCurrentPosition();
        error = targetPosition - currentPosition;
        if (Math.abs(error)<100) {
            spinController.reset();
        }
        power = Range.clip(spinController.calculate(0, error), -1, 1);

        s1.setPower(power);
        s2.setPower(power);
    }

    public void setPID(double p, double i, double d) {
        spinController.setPID(p, i, d);
    }

    public boolean isFinished() {
        return Math.abs(error) < 190;
    }

    public int spindexerShoot() {
        //ball is out
        if (colorSensor.getDistance(DistanceUnit.CM)>=7.3 && isFinished()) {
            spinController.reset();
            nextSlot();
            ballsShot++;
        }
        return ballsShot;
    }

    public void setBallsEatenToZero() {
        ballsEaten = 0;
    }
    public void autoIntake() {
        if (ballsEaten == 3) {
            return;
        }
        if (colorSensor.getDistance(DistanceUnit.CM)<=5 && isFinished()) {
            spinController.reset();
            slots[ballsEaten] = getColor();
            ballsEaten++;
            nextSlot();
        }
    }

    public void nextSlot() {
        toUpdate = true;
        setTargetPosition(targetPosition+slotIncrement);
    }
    public void prevSlot() {
        toUpdate = true;
        setTargetPosition(targetPosition-slotIncrement);
    }

    public void holdAtZero() {
        toUpdate = true;
        ballsShot = 0;
        int currPos = intake.getCurrentPosition();
        int position = slotIncrement + currPos - (currPos % slotIncrement);
        setTargetPosition(position);
    }
    private int getIndex() {
        index = (intake.getCurrentPosition()/slotIncrement) % 3;
        return index;
    }

    public void reset() {
        spinController.reset();
    }

    public void shootSpindexer() {
        s1.setPower(0.5);
        s2.setPower(0.5);
    }

    public SlotState getColor() {
        double red = (double) colorSensor.red()/colorSensor.alpha();
        double green = (double) colorSensor.green()/colorSensor.alpha();
        double blue = (double) colorSensor.blue()/colorSensor.alpha();

        double min = Math.min(red, Math.min(green, blue));
        red /= min;
        green /= min;
        blue /= min;
//        if (red == 1 && 3.7 <= green && green <= 4.5 && 2.8 <= blue && blue <= 3.1) {
//            currentState = G;
//        } else if (red == 1 && 1 <= green && green <= 1.5 && 1.7 <= blue && blue <= 2) {
//            currentState = P;
//        }

        if (green<=2.25) {
            currentState = P;
        } else  {
            currentState = G;
        }

        return currentState;
    }

    public String getTelemetry() {
        getColor();
        index = (int)Math.round(Math.abs((double)intake.getCurrentPosition()/slotIncrement)) %3;
        return "Current Position: " + intake.getCurrentPosition() + "\n" +
                "\nTarget Position: " + targetPosition +
                "\nError: " + error +
                "\nIsFinished: " + isFinished() +
                "\nToUpdate: " + toUpdate +
                "\nPower: " + power +
                "\n\nCurrentState: " + currentState +
                "\nSlots: " + Arrays.toString(slots) +
                "\nMotif: " + Arrays.toString(motif) +
                "\nBalls Eaten: " + ballsEaten;
    }

    public void setSpinPower(double pw) {
        spinPower = pw;
    }

    public void setMotif(SlotState[] motif) {
        this.motif = motif;
    }

    public static SlotState[] switchSlot(SlotState[] slots) {
        if (Arrays.equals(slots, new SlotState[]{P, P, G})) {
            slots = new SlotState[]{P, G, P};
        }
        else if (Arrays.equals(slots, new SlotState[]{P, G, P})) {
            slots = new SlotState[] {G, P, P};
        }
        else if (Arrays.equals(slots, new SlotState[]{G, P, P})) {
            slots = new SlotState[]{P, P, G};
        }
        return slots;
    }

    public static SlotState[] switchSlotAlt(SlotState[] slots) {
        int Gposition = 0;
        while (slots[Gposition++] != G) {
            ;
        }
        Gposition++;
        Gposition %= 3;
        for (int i = 0; i < 3; i++) {
            slots[i] = (i == Gposition) ? G : P;
        }
        return slots;
    }
    public void sort() {
        int sortCount = 0;
        while (!Arrays.equals(slots, motif)) {
            slots = switchSlot(slots);
            sortCount++;
            if (sortCount == 3) {
                break;
            }
        }

        if (sortCount == 1) {
            nextSlot();
        }
        else if (sortCount == 2) {
            prevSlot();
        }

    }
}
