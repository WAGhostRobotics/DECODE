package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoUtil.LoopRateTracker;
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Gus;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    public static boolean blue = false;
    public static int shooterThreshold = 5;
    public static double distanceThreshold = 12.0;
    public static double hoodK = 0.003;
    public static double xTranslation = 1.76, yTranslation = 1.3;
    public static double tP = 0.000032, tI = 0.00000004, tD = 0;
    public static double turretKStatic = 0.026;
    public static boolean moving = false;
    LoopRateTracker loopRateTracker;

    protected Pose2D failSafePose = new Pose2D(DistanceUnit.INCH, 69.03, 83.0, DEGREES, 180);
    boolean adjustingHood = false;



    @Override
    public void runOpMode() throws InterruptedException {
        loopRateTracker = new LoopRateTracker();
        boolean full = false;
        boolean wasEmpty = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 1;
        ElapsedTime shootTimer, driveTimer;
        shootTimer = new ElapsedTime();
        driveTimer = new ElapsedTime();
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader extremeFailsafe = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.B);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);



        waitForStart();
        Gus.init(hardwareMap, blue, true);
        Gus.localizer.setHeadingDegrees(180);
        Gus.intake.closeGate();

        while (opModeIsActive()) {
            loopRateTracker.updateLoopRate();
            Gus.shooter.setTurretKStatic(turretKStatic);
            Gus.shooter.setTurretPID(tP, tI, tD);
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
            if (distance > 3.2) {
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
                    Gus.shooter.setHood(Gus.shooterLUT.getHoodAngle(distance), adjustingHood);
                    Gus.shooter.setTargetVelocity(Gus.shooterLUT.getSpeed(distance));
                    Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(Gus.limelight.getTurretAngle()));
                }
                else {
                    Gus.shooter.setTurretTargetPos(0);
                    Gus.shooter.setTargetVelocity(182);
                    Gus.shooter.setHood(0.17);
                }
            }
            else {
                Gus.shooter.setTargetVelocity(0);
                Gus.shooter.setTurretTargetPos(0);
            }
            Gus.shooter.updateShooter();
            Gus.shooter.updateTurret();

            Gus.intake.updateIntake();

            if (Gus.intake.isFull() && !full) {
                full = true;
                gamepad1.rumble(200);
                Gus.shooter.resetTurret();
            }

            if (gateReader.wasJustReleased()) {
                if (shooting) {
                    Gus.limelight.setLocalizer();
                }
                full = false;
                wasEmpty = true;
                shooting = false;
                Gus.intake.setBallIn(false);
                Gus.intake.closeGate();
                Gus.shooter.resetTurret();
                Gus.shooter.setTurretTargetPos(0);
            }

            x = -gamepad1.left_stick_y;
            y = -gamepad1.left_stick_x;
            driveTurn = -gamepad1.right_stick_x;
            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            heading = Gus.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);

            if (Math.abs(magnitude) <= 0.1 && Math.abs(driveTurn) <= 0.1 && gamepad1.right_bumper ) {
                if (initialized && driveTimer.seconds() > 1) {
                    MotionPlanner.holdPosition(targetX, targetY, targetHeading);
                }
                else {
                    targetX = Gus.localizer.getPosX();
                    targetY = Gus.localizer.getPosY();
                    targetHeading = Gus.localizer.getHeading();
                }
            }
            else {
                initialized = true;
                Gus.drivetrain.drive(magnitude, theta, driveTurn, 1);
                driveTimer.reset();
            }



            if (gamepad1.left_trigger>0.3) {
                Gus.intake.rollerOut();
            }
            else if (gamepad2.right_bumper) {
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


//            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
//            telemetry.addData("Distance: ", Gus.limelight.getTelemetry());
//            telemetry.addData("X: ", Gus.localizer.getPosX());
//            telemetry.addData("Y: ", Gus.localizer.getPosY());
//            telemetry.addData("Heading: ", Gus.localizer.getHeading());
//            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
//            telemetry.addData("Timer: ", shootTimer.seconds());
            telemetry.addData("LoopRate: ", loopRateTracker.getLoopRateHz());
            telemetry.update();

        }



    }
}
