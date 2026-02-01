package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp;

@TeleOp
public class BlueTeleOp extends MainTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.blue = true;
        super.runOpMode();
    }
}
