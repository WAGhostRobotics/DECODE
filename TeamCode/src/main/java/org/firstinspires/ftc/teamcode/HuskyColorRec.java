package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "huskyLens")
public class HuskyColorRec extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HuskyLens huskyLens;
        int GREEN_BALL_ID = 2;
        int PURPLE_BALL_ID= 1;
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addLine("Initialized and ready. Waiting for start.");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            HuskyLens.Block[] blocks = huskyLens.blocks();
            int xGreen = -1, yGreen = -1;
            int xPurple = -1, yPurple = -1;

            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Ball Found", block.toString());
                if (block.id == GREEN_BALL_ID) {
                    xGreen = block.x;
                    yGreen = block.y;
                } else if (block.id == PURPLE_BALL_ID) {
                    xPurple = block.x;
                    yPurple = block.y;
                }
            }

            // Report positions on telemetry
            if (xGreen != -1 && yGreen != -1) {
                telemetry.addData("Green Ball", "x = %d y = %d", xGreen, yGreen);
            } else {
                telemetry.addData("Green Ball", "Not Found");
            }

            if (xPurple != -1 && yPurple != -1) {
                telemetry.addData("Purple Ball", "x = %d y= % d", xPurple, yPurple);
            } else {
                telemetry.addData("Purple Ball", "Not Found");
            }

            telemetry.update();

        }
    }
}
