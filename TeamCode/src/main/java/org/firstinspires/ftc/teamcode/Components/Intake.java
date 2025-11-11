package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.EMPTY;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.GREEN;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.PURPLE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.ml.EM;

public class Intake {
    CRServo s1, s2;
    DcMotorEx intake;
    RevColorSensorV3 colorSensor;
    public static int slotIncrement = 2677;
    double spinPower = 0.8;
    double intakePower = 0.8;
    int index;
    SlotState currentState = EMPTY;

    public enum SlotState {
        PURPLE,
        GREEN,
        EMPTY
    }
    SlotState[] slots = {GREEN, PURPLE, PURPLE};

    public Intake(HardwareMap hwMap) {
        index = 0;
        intake = hwMap.get(DcMotorEx.class, "intake");
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
        s1.setPower(spinPower);
        s2.setPower(spinPower);
    }

    public void rotateCCW() {
        s1.setPower(-spinPower);
        s2.setPower(-spinPower);
    }

    public void stopSpindexer() {
        s1.setPower(0);
        s2.setPower(0);
    }


    private int getIndex() {
        index = (intake.getCurrentPosition()/slotIncrement) % 3;
        return index;
    }

    public void shootSpindexer() {
        s1.setPower(0.5);
        s2.setPower(0.5);
    }

    public void getColor() {
        double red = (double) colorSensor.red()/colorSensor.alpha();
        double green = (double) colorSensor.green()/colorSensor.alpha();
        double blue = (double) colorSensor.blue()/colorSensor.alpha();

        double min = Math.min(red, Math.min(green, blue));
        red /= min;
        green /= min;
        blue /= min;
        if (colorSensor.getDistance(DistanceUnit.CM)<3.2) {
            if (red == 1 && 3.7 <= green && green <= 4.5 && 2.8 <= blue && blue <= 3.1) {
                currentState = GREEN;
            } else if (red == 1 && 1 <= green && green <= 1.5 && 1.7 <= blue && blue <= 2) {
                currentState = PURPLE;
            }
        }
        else {
            currentState = EMPTY;
        }
    }

    public String getTelemetry() {
        getColor();
        index = (int)Math.round(Math.abs((double)intake.getCurrentPosition()/slotIncrement)) %3;
        return "Current State: " + currentState + "\n" +
                "Current Position: " + intake.getCurrentPosition() + "\n" +
                "Index: " + index;
    }
}
