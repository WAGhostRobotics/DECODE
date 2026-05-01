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
import org.firstinspires.ftc.teamcode.AutoUtil.MotionPlanner;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Core.Walt;

import java.io.File;
import java.util.List;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    public static int multiplier = 1;

    public static boolean blue = false;
    boolean initialized = false;
    public static double hoodK = 0.004;
    public static double xTranslation = 1.74, yTranslation = 1.2;
    public static double tP = 0.00002, tI = 0.000000, tD = 0;

    public static int visionDelay = 50;
    public static int pidTimerDelay =30;
    public static boolean moving = false;
    public static int targetVelocity = 0;
    public static double hoodPos = 0;

    int failsafeTargetVelocity = 163;
    public static double limelightVelThreshold = 0.5;
    public static double relocalizeVelThreshold = 2;
    double failsafeHoodPos = 0.38;
    double gateTimerThreshold = 0.45;
    LoopRateTracker loopRateTracker;

    protected Pose2D failSafePose = new Pose2D(DistanceUnit.INCH, 69.03, 83.0, DEGREES, 180);
    double parkHeading = 0;
    File file;
    boolean slowMo = false;
    double newHeading = 0;
    ElapsedTime initializedTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        loopRateTracker = new LoopRateTracker();
        boolean parking = false;
        boolean full = false;
        boolean adjustingHood = false;
        boolean wasEmpty = false;
        boolean readyToShoot = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 0.3;
        ElapsedTime shootTimer;
        ElapsedTime visionTimer;
        ElapsedTime pidTimer;
        ElapsedTime fullTimer;
        pidTimer = new ElapsedTime();
        fullTimer = new ElapsedTime();
        visionTimer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        ToggleButtonReader parkButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);

        ToggleButtonReader extremeFailsafe = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.B);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);
        ToggleButtonReader slowMoButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.A);
        ToggleButtonReader stopIntakeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.Y);

        double prevHeading = Double.parseDouble(ReadWriteFile.readFile(file));
        if (!blue) {
            newHeading = prevHeading + 180;
        }
        else {
            newHeading = prevHeading;
        }
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeInInit()) {
            Walt.initCamera(hardwareMap, blue);
            telemetry.addData("Heading: ", newHeading);
            telemetry.update();
            Walt.limelight.start();
        }
        initialized = false;
        waitForStart();
        Walt.init(hardwareMap, blue, true);
        Walt.intake.closeGate();
        initializedTimer.reset();

        while (opModeIsActive()) {
//            moving = gamepad2.left_bumper;
            Walt.localizer.update();



            if (!initialized) {
                if (initializedTimer.seconds() > 0.3) {
                    initialized = true;
                }
                Walt.localizer.setHeadingDegrees(newHeading);
            }

            loopRateTracker.updateLoopRate();
//            Gus.shooter.setShooterThreshold(shooterThreshold);
//            Walt.shooter.setHoodAdjustmentConstant(hoodK);
            Walt.limelight.setLimelightVelocityThreshold(limelightVelThreshold);
            Walt.limelight.setRelocalizeVelocityThreshold(relocalizeVelThreshold);
            Walt.limelight.setXYTranslation(xTranslation, yTranslation);

            if (failsafeButton.wasJustReleased()) {
                Walt.limelight.turnOff();
                Walt.localizer.setPose(failSafePose);
            }

            if (extremeFailsafe.wasJustReleased()) {
                failsafe = !failsafe;
            }


            // Testing tracking always
            if (Walt.intake.isOneBallIn() && visionTimer.milliseconds() > visionDelay) {
                Walt.limelight.trackAprilTag(Walt.localizer.getHeading()-180, Walt.shooter.getTurretAngle(), moving);
                visionTimer.reset();
            }
            double distance = Walt.limelight.getDistance();
            if (distance > 2.0) {
                adjustingHood = true;
            }
            else {
                adjustingHood = false;
            }

            if (Walt.intake.isOneBallIn() && !readyToShoot) {
//                if (wasEmpty) {
//                    shootTimer.reset();
//                    wasEmpty = false;
//                } else if (shootTimer.seconds() > 0.5) {
//                    //
//                }
                shootTimer.reset();
            }
//            else {
//                shootTimer.reset();
//            }


            if (Walt.intake.isOneBallIn()) {
                if (!failsafe) {
                    if (targetVelocity == 0)
                        Walt.shooter.setTargetVelocity(Walt.limelight.getFlywheelVelocity());
                    else
                        Walt.shooter.setTargetVelocity(targetVelocity);

                    if (hoodPos == 0)
                        Walt.shooter.setHood(Walt.shooterLUT.getHoodAngle(distance), adjustingHood);
                    else
                        Walt.shooter.setHood(hoodPos, adjustingHood);

                    Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(Walt.limelight.getTurretAngle()));
                }
                else {
                    Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
                    Walt.shooter.setTargetVelocity(failsafeTargetVelocity);
                    Walt.shooter.setHood(failsafeHoodPos);
                }
            }
            else {
                Walt.shooter.setTargetVelocity(0);
                Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
            }

            if (pidTimer.milliseconds() > pidTimerDelay) {
                Walt.shooter.updateShooter();
                pidTimer.reset();
            }

            if (gateReader.wasJustReleased()) {
                gamepad2.rumble(200);
                full = false;
                wasEmpty = true;
                shooting = false;
                readyToShoot = false;
                Walt.intake.setBallIn(false);
                Walt.intake.closeGate();
                Walt.shooter.resetTurret();
                Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
                Walt.ledLights.redColor();
            }


            Walt.intake.updateIntake();

            if (Walt.intake.isFull()) {
                if (!full) {
                    full = true;
                    gamepad1.rumble(200);
                    Walt.ledLights.blueColor();
                    fullTimer.reset();
                }
                if (fullTimer.seconds() > gateTimerThreshold) {
                    Walt.intake.openGate();
                    readyToShoot = true;
                }
//                Gus.shooter.resetTurret();
            }
            else {
                if (!shooting && Walt.intake.gateOpen) {
                    Walt.intake.closeGate();
                }
                fullTimer.reset();
            }


            x = -gamepad1.left_stick_y;
            y = -gamepad1.left_stick_x;


            if (parking) {
                driveTurn = MotionPlanner.holdHeading(parkHeading);
            }
            else {
                driveTurn = -gamepad1.right_stick_x;
            }

            if (parkButton.wasJustPressed()) {
                if (blue) {
                    Walt.ledLights.redColor();
                }
                else {
                    Walt.ledLights.blueColor();
                }
                parking = true;
            }
            else if (Math.abs(gamepad1.right_stick_x) >= 0.5) {
                parking = false;
            }

            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            heading = Walt.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);

            Walt.drivetrain.drive(magnitude, theta, driveTurn, 1);




            if (gamepad1.left_trigger>0.3) {
                Walt.intake.rollerOut();
            }
            else if (gamepad1.left_bumper) {
                Walt.intake.rollerStop();
            }
            else if (gamepad1.right_trigger > 0.2) {
                Walt.intake.bruteRollerIn();
            }
            else if (gamepad2.right_bumper) {
                if (shootButton.wasJustPressed()) {

                    shooting = true;
                    readyToShoot = true;
                    Walt.intake.rollerStop();
                    Walt.intake.loaderStop();
                    Walt.intake.setBallIn(true);
                    Walt.intake.openGate();
                }
                else if (shootTimer.seconds() > delay) {
                    if (!slowMo) {
                        Walt.shooter.shoot();
                    }
                    else {
                        Walt.shooter.shootSlowMotion();
                    }
                }
                else {
                    Walt.intake.rollerStop();
                    Walt.shooter.stop();
                }
            }
            else {
                if (!shooting)
                    Walt.intake.rollerIn();
                else {
                    Walt.intake.rollerStop();
                }
            }

            if (shootButton.wasJustPressed()) {
                Walt.limelight.setLocalizer(Walt.localizer.getHeading()-180, Walt.shooter.getTurretAngle(), true);
            }



            if (stopIntakeButton.wasJustReleased()) {
                Walt.intake.setFull();
                full = true;
            }


            gateReader.readValue();
            shootButton.readValue();
            imuReader.readValue();
            extremeFailsafe.readValue();
            failsafeButton.readValue();
            parkButton.readValue();
            stopIntakeButton.readValue();

            if (imuReader.wasJustReleased()) {
                Walt.limelight.resetInitialized();
                Walt.localizer.setHeadingDegrees(180);
            }


//            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
//            telemetry.addData("Shooter: ", Walt.shooter.getTelemetry());
//            telemetry.addData("Shooter Timer: ", shootTimer.seconds());
            telemetry.addData("Limelight\n", Walt.limelight.getPositions());
//            telemetry.addData("Localizer X: ", Gus.localizer.getPosX());
//            telemetry.addData("Localizer Y: ", Gus.localizer.getPosY());
            telemetry.addData("X: ", Walt.localizer.getPosX());
            telemetry.addData("Y: ", Walt.localizer.getPosY());
//            telemetry.addData("Is Limelight chilling: ", Gus.limelight.isAlive());
//            telemetry.addData("Heading: ", Walt.localizer.getHeading());
//            telemetry.addData("Initialized: ", initialized);
//            telemetry.addData("Intake: ", Walt.intake.getTelemetry());
//            telemetry.addData("Timer: ", shootTimer.seconds());
//            telemetry.addData("Lim: ", Walt.limelight.getDistance());
            telemetry.addData("LoopRate: ", loopRateTracker.getLoopRateHz());
//            telemetry.addData("ID: ", Gus.limelight.getFiducialID());
            telemetry.update();

        }



    }
}