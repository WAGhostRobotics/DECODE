package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

import java.io.File;
import java.util.List;

@Config
@TeleOp
public class OneGamepadTeleop extends LinearOpMode {
    public static boolean blue = false;
    boolean initialized = false;
    public static int shooterThreshold = 3;
    public static double antiShootPower = -0.08;
    public static double hoodK = 0.003;
    public static double xTranslation = 1.76, yTranslation = 1.3;
    public static double tP = 0.00002, tI = 0.000000, tD = 0;
    public static double tPF = 0.0000, tIF = 0.00000, tDF = 0;
    public static double turretKStatic = 0.04;
    public static double fullPowerThreshold = 5000;
    public static double shootTimeConstant = 1.7;
    public static double velocityConstant = 0;
    public static boolean moving = false;
    public static int targetVelocity;
    public static double hoodPos;
    LoopRateTracker loopRateTracker;

    protected Pose2D failSafePose = new Pose2D(DistanceUnit.INCH, 69.03, 83.0, DEGREES, 180);
    public static double distanceThreshold = 2.0;

    File file;


    @Override
    public void runOpMode() throws InterruptedException {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        loopRateTracker = new LoopRateTracker();
        boolean full = false;
        boolean adjustingHood = false;
        boolean wasEmpty = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 1;
        ElapsedTime shootTimer;
        shootTimer = new ElapsedTime();
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader extremeFailsafe = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.START);
        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);


        double prevHeading = Double.parseDouble(ReadWriteFile.readFile(file));


        waitForStart();
        Gus.init(hardwareMap, blue, true);
        Gus.intake.closeGate();

        while (opModeIsActive()) {
            if (!initialized) {
                initialized = true;
                Gus.localizer.setHeadingDegrees(prevHeading+90);
            }
            loopRateTracker.updateLoopRate();
            Gus.intake.setAntiShootPower(antiShootPower);
            Gus.limelight.setVelocityConstant(velocityConstant);
            Gus.limelight.setShootConstant(shootTimeConstant);
            Gus.shooter.setShooterThreshold(shooterThreshold);
            Gus.shooter.setHoodAdjustmentConstant(hoodK);
            Gus.limelight.setXYTranslation(xTranslation, yTranslation);

            if (failsafeButton.wasJustReleased()) {
                Gus.limelight.turnOff();
                Gus.localizer.setPose(failSafePose);
            }

            if (extremeFailsafe.wasJustReleased()) {
                failsafe = !failsafe;
            }

            Gus.localizer.update();
            if (Gus.intake.isOneBallIn()) {
                Gus.limelight.trackAprilTag(Gus.localizer.getHeading()-180, Gus.shooter.getTurretAngle(), moving);
            }
            double distance = Gus.limelight.getDistance();
            if (distance > distanceThreshold) {
                adjustingHood = true;
            }
            else {
                adjustingHood = false;
            }

            if (Gus.intake.isOneBallIn()) {
                if (wasEmpty) {
                    shootTimer.reset();
                    wasEmpty = false;
                } else if (shootTimer.seconds() > 0.5) {
                    Gus.intake.openGate();
                }
            }
            else {
                shootTimer.reset();
            }


            if (Gus.intake.isOneBallIn()) {
                if (!failsafe) {
                    if (targetVelocity == 0)
                        Gus.shooter.setTargetVelocity(Gus.limelight.getFlywheelVelocity());
                    else
                        Gus.shooter.setTargetVelocity(targetVelocity);

                    if (hoodPos == 0)
                        Gus.shooter.setHood(Gus.shooterLUT.getHoodAngle(distance), adjustingHood);
                    else

                        Gus.shooter.setHood(hoodPos, adjustingHood);

                    Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
                }
                else {
                    Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
                    Gus.shooter.setTargetVelocity(targetVelocity);
                    Gus.shooter.setHood(hoodPos);
                }
            }
            else {
                Gus.shooter.setTargetVelocity(0);
                Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
            }
            Gus.shooter.updateShooter();
            Gus.shooter.updateTurret();

            Gus.intake.updateIntake();

            if (Gus.intake.isFull() && !full) {
                full = true;
                gamepad1.rumble(200);
//                Gus.shooter.resetTurret();
            }

            if (gateReader.wasJustReleased()) {
                full = false;
                wasEmpty = true;
                shooting = false;
                Gus.intake.setBallIn(false);
                Gus.intake.closeGate();
                Gus.shooter.resetTurret();
                Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
                Gus.intake.setRampFullThreshold();
            }

            x = -gamepad1.left_stick_y;
            y = -gamepad1.left_stick_x;
            driveTurn = -gamepad1.right_stick_x;
            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            heading = Gus.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);

            Gus.drivetrain.drive(magnitude, theta, driveTurn, 0.95);




            if (gamepad1.left_trigger>0.3) {
                Gus.intake.rollerOut();
            }
            else if (gamepad1.right_trigger > 0.2) {
                Gus.intake.bruteRollerIn();
            }
            else if (gamepad1.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    shooting = true;
                    Gus.intake.rollerStop();
                    Gus.intake.setBallIn(true);
                    Gus.intake.openGate();
                }
                else if (shootTimer.seconds() > delay) {
                    Gus.shooter.shoot();
                }
                else {
                    Gus.intake.rollerStop();
                    Gus.shooter.stop();
                }
            }
            else {
                if (!shooting)
                    Gus.intake.rollerIn();
                else {
                    Gus.intake.rollerStop();
                }
            }

            if (shootButton.wasJustReleased()) {
                Gus.limelight.setLocalizer(Gus.localizer.getHeading() - 180, Gus.shooter.getTurretAngle());
            }

            if (Gus.intake.gateOpen) {
                gamepad1.setLedColor(255, 0, 0, 5);
            }
            else {
                gamepad1.setLedColor(0, 255, 0, 5);
            }


            gateReader.readValue();
            shootButton.readValue();
            imuReader.readValue();
            extremeFailsafe.readValue();
            failsafeButton.readValue();


            if (imuReader.wasJustReleased()) {
                Gus.limelight.resetInitialized();
                Gus.localizer.setHeadingDegrees(180);
            }


            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
//            telemetry.addData("Shooter: ", Gus.shooter.getTelemetry());
            telemetry.addData("Distance: ", Gus.limelight.getTelemetry());
//            telemetry.addData("X: ", Gus.localizer.getPosX());
//            telemetry.addData("Y: ", Gus.localizer.getPosY());
//            telemetry.addData("Heading: ", Gus.localizer.getHeading());
            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
//            telemetry.addData("Timer: ", shootTimer.seconds());
            telemetry.addData("LoopRate: ", loopRateTracker.getLoopRateHz());
            telemetry.update();

        }



    }
}
