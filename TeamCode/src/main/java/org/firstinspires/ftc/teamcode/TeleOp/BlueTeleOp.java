package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp;

@TeleOp
@Config
public class BlueTeleOp extends MainTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.multiplier = -1;
        blue = true;
        super.runOpMode();
    }
}
