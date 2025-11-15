package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.G;
import static org.firstinspires.ftc.teamcode.Components.Intake.SlotState.P;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
public class MotifTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.start();
        limelight3A.pipelineSwitch(2);
        int motifID;
        Intake.SlotState[] motif;
        waitForStart();
        while (opModeIsActive()) {
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {       // If April tag is visible
                motifID = llResult.getFiducialResults().get(0).getFiducialId();
                if (motifID == 21) {
                    motif = new Intake.SlotState[]{G, P, P};
                }
                else if (motifID == 22) {
                    motif = new Intake.SlotState[]{P, G, P};
                }
                else if (motifID == 23) {
                    motif = new Intake.SlotState[]{P, P, G};
                }
                telemetry.addData("Limelight Result: ", llResult.getFiducialResults());
                telemetry.addData("Limelight Result[0]: ", llResult.getFiducialResults().get(0));
                telemetry.addData("ID: ", llResult.getFiducialResults().get(0).getFiducialId());
            }
            telemetry.update();
        }
    }
}
