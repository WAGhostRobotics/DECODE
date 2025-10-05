package org.firstinspires.ftc.teamcode.RI3W;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
@TeleOp
public class AutoShoot extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(AutoShoot.class);
    public static boolean tuning = false;
    private final double theta = 55;
    private final double experimentalSlope = 0.822;

    public static double shooterVelocity;
    public static double intakePower;

    public static double xTranslation = 1.482;
    public static double yTranslation = 1.413; // Y is only for Blue. Red would be negative
    public static double P = 0.009, I = 0, D = 0;
    public static double kStaticTurn = 0.1;
    PIDController headingControl;
    boolean shooterOn = false;
    boolean isFieldOriented = true;
    private double error;

    @Override
    public void runOpMode() throws InterruptedException {
        ToggleButtonReader shooterButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader fieldOriented = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        ToggleButtonReader resetHeading = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        headingControl = new PIDController(P, I, D);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        double driveTurn = 0;
        double aprilX, aprilY, distance;
        limelight3A.start();
        limelight3A.pipelineSwitch(0);
        George.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            George.localizer.update();
//            limelight3A.updateRobotOrientation(George.localizer.getHeading());
            LLResult llResult = limelight3A.getLatestResult();
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double magnitude = Math.hypot(x, y);
            double tx = 0;
            double ta = 0;
            double theta = Math.toDegrees(Math.atan2(y, x));
            if (isFieldOriented) {
                theta = normalizeDegrees(theta - George.localizer.getHeading());
            }
            headingControl.setPID(P, I, D);
            if (llResult != null && llResult.isValid() && shooterOn) {
                gamepad1.setLedColor(0, 255, 0, 1000);
                Pose3D botPose = llResult.getBotpose();
                aprilX = (botPose.getPosition().x + xTranslation);
                aprilY = (botPose.getPosition().y + yTranslation);
                distance = Math.hypot(aprilX, aprilY);
                tx = -llResult.getTx()+1.7;     // offset bc limelight not in middle
                ta = llResult.getTa();
                if (Math.abs(tx)<=0.75) {
                    tx = 0;
                }
                driveTurn = headingControl.calculate(0, tx);
                driveTurn = Math.signum(driveTurn)*kStaticTurn + driveTurn;
                driveTurn = Range.clip(driveTurn, -1, 1);

                if (!tuning)
                    shooterVelocity = computeVelocity(distance);
                telemetry.addData("Pose: ", botPose);
                telemetry.addData("AprilX: ", aprilX);
                telemetry.addData("AprilY: ", aprilY);
                telemetry.addData("Distance: ", distance);
            }
            else {
                driveTurn = -gamepad1.right_stick_x;
            }
            if (shooterButton.wasJustReleased()) {
                shooterOn = !shooterOn;
            }
            if (fieldOriented.wasJustReleased()) {
                isFieldOriented = !isFieldOriented;
            }
            if (resetHeading.wasJustReleased()) {
                George.localizer.resetHeading();
            }

            if (shooterOn)
                George.shooter.setTargetVelocity(shooterVelocity);
            else {
                George.shooter.resetPID();
                George.shooter.setTargetVelocity(0);
            }

            if (gamepad1.right_trigger>0) {
                George.shooter.setIntake(1);
            }
            else if (gamepad1.left_trigger > 0) {
                George.shooter.setIntake(-0.55);
            }
            shooterButton.readValue();
            fieldOriented.readValue();
            resetHeading.readValue();
            George.drivetrain.drive(magnitude, theta, driveTurn, 0.875);
            George.shooter.updateShooter();
            George.shooter.setIntake(intakePower);
            telemetry.addData("ShooterVelocity: ", shooterVelocity);
            telemetry.addData("Heading: ", George.localizer.getHeading());
            telemetry.addData("Tx: ", tx);
            telemetry.addData("Ta: ", ta);
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.update();
        }
    }


    public double distanceFunction(double x, double theta) {  // Theta in degrees
        double xSquared = Math.pow(x, 2);
        double tanTheta = Math.tan(Math.toRadians(theta));   // Theta converted to radians before than
        return (xSquared)/((x*tanTheta)-1);
    }

    public double computeVelocity(double rawX) {
        // Haha Caveat
        if (rawX < 2.0) {
            return 125;
        }
        else if (rawX > 3.6) {
            return 170;
        }
        double x = distanceFunction(rawX, theta);
        double rawY = experimentalSlope*x;
        return Math.sqrt(rawY)*100;     // Velocity
    }
}
