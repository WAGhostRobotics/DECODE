package org.firstinspires.ftc.teamcode.AutoUtil;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RI3W.George;

@TeleOp
@Config
public class Tuner extends LinearOpMode {
    public static double xP=0.04, xI=0.003, xD=0;
    public static double yP=0.04, yI=0.006, yD=0;
    public static double hP=0.012, hI=0.0003, hD;
    public static double kStaticX = 0.15;
    public static double kStaticY = 0.24;
    public static double kStaticTurn = 0.12;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double heading = 0;
    Bezier path;
    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader startPath = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        path = new Bezier(0,
                new Point(targetX, targetY)
        );
        George.init(hardwareMap);
        MotionPlanner follower = new MotionPlanner(George.drivetrain, George.localizer, hardwareMap);
        follower.setMovementPower(0.9);
        follower.startFollowingPath(path);
        waitForStart();
        while (opModeIsActive()) {
            double x = George.localizer.getPosX();
            double y = George.localizer.getPosY();
            path = new Bezier(
                    heading,
                    new Point(x, y),
                    new Point(targetX, targetY)
            );
            if (startPath.wasJustReleased()) {
                follower.setXPID(xP, xI, xD);
                follower.setYPID(yP, yI, yD);
                follower.setHeadingPID(hP, hI, hD);
                follower.setXKStatic(kStaticX);
                follower.setYKStatic(kStaticY);
                follower.setKStaticTurn(kStaticTurn);
                follower.startFollowingPath(path);
            }
            startPath.readValue();
            follower.update();
            George.localizer.update();
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
        }
    }
}
