package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 ds = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        waitForStart();

        while(opModeIsActive()){
            double red = (double)ds.red()/ds.alpha();
            double green = (double)ds.green()/ds.alpha();
            double blue = (double)ds.blue()/ds.alpha();

            double min = Math.min(red, Math.min(green, blue));
            red /= min;
            green /= min;
            blue /= min;

            telemetry.addData("Color is: ", red);
            telemetry.addData("Color is: ", green);
            telemetry.addData("Color is: ", blue);
            telemetry.addData("Distance is: ", ds.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}
