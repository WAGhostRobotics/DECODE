package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class pidtrunk extends LinearOpMode{

    public static double p, i, d;
    public static int targetPos1;

    @Override
    public void runOpMode() throws InterruptedException{
        TrunkOrTreat t = new TrunkOrTreat(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            t.setPID(p, i, d);
            t.setTargetPosition(targetPos1);
            t.update();

//            telemetry.addData("rotation pos: ", base.getCurrentPosition());
//            telemetry.addLine("J1 target position is " + t.j1TargetPosition);
            telemetry.addData("J1 current pos: ", t.first.getCurrentPosition());
////            telemetry.addLine("J2 target position is " + t.j2TargetPosition);
            telemetry.addData("J2 current pos: ", t.second.getCurrentPosition());
//            telemetry.addData("wrist pos: ", t.wrist.getPosition());
            telemetry.addData("power 1 = ", t.power1);
            telemetry.addData("power 2 = ", t.power2);
            telemetry.update();
        }
    }
}
