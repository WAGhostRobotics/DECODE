package org.firstinspires.ftc.teamcode.RI3W;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevDistanceSensor {

    private DistanceSensor distance;

    public RevDistanceSensor(HardwareMap hardwareMap){
        distance = hardwareMap.get(DistanceSensor.class, "distance");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.CM);
    }
}
