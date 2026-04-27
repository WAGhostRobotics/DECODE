package org.firstinspires.ftc.teamcode.TestingTeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Config
public class PedroLocalizerTest extends OpMode {
    Follower follower;
    Pose startingPose = new Pose(54.5, 113.0,Math.toRadians(180));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double magnitude = Math.hypot(x, y);
        double theta = Math.toDegrees(Math.atan2(y, x));
        follower.update();

//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));



        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
//        ReadWriteFile.writeFile(file, Double.toString(Gus.localizer.getHeading()));
    }
}
