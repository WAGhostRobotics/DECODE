package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class BrushlandDistREV extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "intakeDistance");
// set the clock speed on this I2C bus to 400kHz:
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        waitForStart();
        while (opModeIsActive()) {
            // read all 3 color channels in one I2C transmission:
            double distance = sensor.getDistance(DistanceUnit.CM);
            telemetry.addData("distance: ", distance);
            telemetry.update();
        }
    }
}
