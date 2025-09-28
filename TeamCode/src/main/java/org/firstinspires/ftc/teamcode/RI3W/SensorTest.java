package org.firstinspires.ftc.teamcode.RI3W;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevDistanceSensor ds = new RevDistanceSensor(hardwareMap);
        Intake intake = new Intake(hardwareMap, gamepad1);
//        RevColorSensor cs = new RevColorSensor(hardwareMap);
        //revColorSensor.DetectedColor detectedColor;

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            telemetry.addData("Distance is: ", ds.getDistance());
            telemetry.update();

            if (ds.getDistance() > 6){
                intake.control();
            } else{
                intake.zeroPower();
            }
            //detectedColor = cs.getDetectedColor(telemetry);
            //telemetry.addData("color detected: ", detectedColor);// check if this works at the end
        }
    }
}
