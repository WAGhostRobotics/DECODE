package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp;

@Config
@TeleOp
public class RedTeleOp extends MainTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        blue = false;
        super.runOpMode();
    }
}
