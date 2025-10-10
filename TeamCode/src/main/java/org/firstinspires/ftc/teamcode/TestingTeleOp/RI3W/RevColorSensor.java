package org.firstinspires.ftc.teamcode.TestingTeleOp.RI3W;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RevColorSensor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public RevColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");
        //colorSensor.setGain(4); // change it based on specimen / green artifact
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns 4 values, RGB+alpha(light)

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("red = ", normRed);
        telemetry.addData("green = ", normGreen);
        telemetry.addData("blue = ", normBlue);


        /**
         * good values are greater than .5 ish
         * red,green,blue
         *
         * purple =
         * green =
         */

        /*
          if(conditions for purple with && statements){
          return DetectedColor.PURPLE;
          } else if (conditions for green with && statements){
          return DetectedColor.GREEN;
          } else{
          return DetectedColor.UNKNOWN;
         */

        return DetectedColor.UNKNOWN;
    }
}
