package org.firstinspires.ftc.teamcode.TestingTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class LiftTest extends OpMode {
    CRServo lift1;
    CRServo lift2;
    double pos = 1;
    @Override
    public void init() {
        lift1 = hardwareMap.get(CRServo.class, "lift1");
        lift2 = hardwareMap.get(CRServo.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger >0.05) {
            lift1.setPower(gamepad1.right_trigger);
            lift2.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.05) {
            lift1.setPower(-gamepad1.left_trigger);
            lift2.setPower(-gamepad1.left_trigger);
        }
        else {
            lift1.setPower(0);
            lift2.setPower(0);
        }

        telemetry.addData("Pos: ", pos);
        telemetry.update();
    }
}
