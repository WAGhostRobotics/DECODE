package org.firstinspires.ftc.teamcode.AutoUtil;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Constants;
import org.firstinspires.ftc.teamcode.Core.Bob;

@TeleOp
@Config
public class Tuner extends LinearOpMode {
    public static double xP= Constants.translationalXP, xI=Constants.translationalXI, xD=Constants.translationalXD;
    public static double yP=Constants.translationalYP, yI=Constants.translationalYI, yD=Constants.translationalYD;
    public static double hP=Constants.headingP, hI=Constants.headingI, hD=Constants.headingD;
    public static double kStaticX = Constants.kStaticX;
    public static double kStaticY = Constants.kStaticY;
    public static double kStaticTurn = Constants.kStaticTurn;
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
        Bob.init(hardwareMap);
        MotionPlanner follower = new MotionPlanner(Bob.drivetrain, Bob.localizer, hardwareMap);
        follower.setMovementPower(0.96);
        follower.startFollowingPath(path);
        waitForStart();
        while (opModeIsActive()) {
            double x = Bob.localizer.getPosX();
            double y = Bob.localizer.getPosY();
            path = new Bezier(
                    heading,
                    new Point(x, y),
                    new Point(targetX, targetY)
            );

            follower.setXPID(xP, xI, xD);
            follower.setYPID(yP, yI, yD);
            follower.setHeadingPID(hP, hI, hD);
            follower.setXKStatic(kStaticX);
            follower.setYKStatic(kStaticY);
            follower.setKStaticTurn(kStaticTurn);

            if (startPath.wasJustReleased()) {

                follower.startFollowingPath(path);
            }
            startPath.readValue();
            follower.update();
            Bob.localizer.update();
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
        }
    }
}
