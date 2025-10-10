package org.firstinspires.ftc.teamcode.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoUtil.Bezier;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.Components.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.MecanumDrive;

@Autonomous
public class NewMPTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
        Bezier b1 = new Bezier(57,
                new Point(0, 0),
                new Point(10, 10),
                new Point(20, 10),
                new Point(40, -10));
        MotionPlanner mp = new MotionPlanner(drive, localizer, hardwareMap);
        mp.setMovementPower(0.8);
        mp.startFollowingPath(b1);
        waitForStart();
        while (opModeIsActive()) {
            mp.update();
            localizer.update();
            telemetry.addData("", mp.getTelemetry());
            telemetry.update();
        }
    }


}
