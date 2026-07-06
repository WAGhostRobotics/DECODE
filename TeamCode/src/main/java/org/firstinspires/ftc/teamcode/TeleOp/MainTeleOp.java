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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    public static double airTime = 0;
    public static double farTransferSpeed = 0.85;

    public static boolean blue = false;
    boolean initialized = false;
    public static double hoodK = 0.002;
    public static double xTranslation = 1.7, yTranslation = 1.25;
    public static int visionDelay = 0;
    public static int pidTimerDelay = 10;
    public static boolean moving = true;
    public static int targetVelocity = 0;
    public static double hoodPos = 0;

    int failsafeTargetVelocity = 163;
    double failsafeHoodPos = 0.38;
    double gateTimerThreshold = 0.45;
    LoopRateTracker loopRateTracker;

    protected Pose2D failSafePose = new Pose2D(DistanceUnit.INCH, 69.03, 83.0, DEGREES, 180);
    double parkHeading = 90;
    File file;
    boolean slowMo = false;
    ElapsedTime initializedTimer = new ElapsedTime();
    public double distConstant = 0;
    public static double closeDistConstant = 0.23;
    public static double sotmDistConstant = 0.37;
    public static double farDistConstant = 0.12;
    double parkAngle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        loopRateTracker = new LoopRateTracker();
        boolean parking = false;
        boolean lifting = false;
        boolean braking = false;
        boolean full = false;
        boolean isFar = false;
        boolean wasEmpty = false;
        boolean readyToShoot = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 0.18;
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
        ToggleButtonReader localizeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_UP);
        ToggleButtonReader extremeFailsafe = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.START);
        ToggleButtonReader failsafeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.B);
        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER);
        ToggleButtonReader stopIntakeButton = new ToggleButtonReader(new GamepadEx(gamepad2), GamepadKeys.Button.Y);
        ToggleButtonReader liftButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        ToggleButtonReader gayButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.Y);

//        double prevHeading = Double.parseDouble(ReadWriteFile.readFile(file));
//        if (!blue) {
//            newHeading = prevHeading + 180;
//        }
//        else {
//            newHeading = prevHeading;
//        }
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Walt.init(hardwareMap, blue, true);

        while (opModeInInit()) {
            loadPose(file);
            telemetry.addData("Heading: ", Walt.localizer.getHeading());
            telemetry.addData("X: ", Walt.localizer.getPosX());
            telemetry.addData("Y: ", Walt.localizer.getPosY());
            Walt.localizer.update();
            telemetry.update();
        }
//        initialized = false;



        waitForStart();
        Walt.intake.closeGate();
        initializedTimer.reset();

        while (opModeIsActive()) {
            Walt.localizer.update();



//            if (!initialized) {
//                if (initializedTimer.seconds() > 0.3) {
//                    initialized = true;
//                }
//                Walt.localizer.setHeadingDegrees(newHeading);
//            }

            loopRateTracker.updateLoopRate();
//            Gus.shooter.setShooterThreshold(shooterThreshold);
            Walt.shooter.setHoodAdjustmentConstant(hoodK);
//            Walt.limelight.setLimelightVelocityThreshold(limelightVelThreshold);
//            Walt.limelight.setRelocalizeVelocityThreshold(relocalizeVelThreshold);
            Walt.limelight.setXYTranslation(xTranslation, yTranslation);

//            if (failsafeButton.wasJustReleased()) {
//                Walt.limelight.turnOff();
//                Walt.localizer.setPose(failSafePose);
//            }

            if (extremeFailsafe.wasJustReleased()) {
                failsafe = !failsafe;
            }


            // Testing tracking always
            Walt.limelight.trackAprilTag(Walt.localizer.getHeading()-180, Walt.shooter.getTurretAngle(), moving);

            double distance = Walt.limelight.getDistance();
            if (distance > 2.8) {
                isFar = true;
                distConstant = farDistConstant;
                Walt.intake.setTransferSpeed(farTransferSpeed);
            }
            else {
                distConstant = closeDistConstant;
                isFar = false;
                Walt.intake.setTransferSpeed(1);
            }

            if (gamepad1.left_trigger > 0.1) {
                distConstant = sotmDistConstant;
            }

            Walt.limelight.setDistanceConstant(distConstant);


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
                    if (targetVelocity == 0) {
                        if (!lifting)
                            Walt.shooter.setTargetVelocity(Walt.limelight.getFlywheelVelocity());
                        else
                            Walt.shooter.setTargetVelocity(0);
                    }
                    else
                        Walt.shooter.setTargetVelocity(targetVelocity);

                    if (hoodPos == 0)
                        Walt.shooter.setHood(Walt.shooterLUT.getHoodAngle(distance), isFar);
                    else
                        Walt.shooter.setHood(hoodPos, isFar);

                    if (!lifting)
                        Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(Walt.limelight.getTurretAngle()));
                }
                else {
                    Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
                    Walt.shooter.setTargetVelocity(failsafeTargetVelocity);
                    Walt.shooter.setHood(failsafeHoodPos);
                }
            }
            else {
                if (!lifting) {
                    Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(Walt.limelight.getTurretAngle()));
                    Walt.shooter.setTargetVelocity(150);
                }
                else {
                    Walt.shooter.setTargetVelocity(0);
                }

            }

            if (pidTimer.milliseconds() > pidTimerDelay) {
                if (isFar) {
                    Walt.shooter.updateShooter();
                }
                else {
                    Walt.shooter.updateShooter();
                }
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
                    Walt.ledLights.redColor();
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

            if (gamepad2.right_bumper) {
                driveTurn = 0;
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





//            if (gamepad1.left_trigger>0.3) {
//                Walt.intake.rollerOut();
//            }
//            else if (gamepad1.left_bumper) {
//                Walt.intake.rollerStop();
//            }

            if (gamepad1.right_trigger > 0.2) {
                Walt.intake.bruteRollerIn();
            }
            else if (gamepad2.right_bumper) {
                if (shootButton.wasJustPressed()) {
                    if (!initialized)
                        Walt.limelight.setLocalizerUsingLimelight(Walt.localizer.getHeading()-180, Walt.shooter.getTurretAngle(), true);
                    initialized = true;
                    shooting = true;
                    readyToShoot = true;
                    Walt.intake.rollerStop();
                    Walt.intake.loaderStop();
                    Walt.intake.setBallIn(true);
                    Walt.intake.openGate();
                }
                else if (shootTimer.seconds() > delay) {
                    if (!slowMo) {
                        Walt.shooter.shootAdaptive(isFar);
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


            if (stopIntakeButton.wasJustReleased()) {
                Walt.intake.setFull();
                full = true;
            }


            if (localizeButton.wasJustPressed()) {
                Walt.limelight.setLocalizerUsingLimelight(Walt.localizer.getHeading()-180, Walt.shooter.getTurretAngle(), true);
                initialized = true;
            }

            if (lifting) {
                Walt.lift.lift();
                Walt.shooter.setTurretTargetPos(Shooter.angleToPosition(parkAngle));
                Walt.shooter.setTargetVelocity(0);
            }
            else if (braking) {
                Walt.lift.brake();
            }
            else {
                Walt.lift.retract();
            }


            gateReader.readValue();
            localizeButton.readValue();
            shootButton.readValue();
            imuReader.readValue();
            extremeFailsafe.readValue();
            failsafeButton.readValue();
            parkButton.readValue();
            stopIntakeButton.readValue();
            liftButton.readValue();
            gayButton.readValue();

            if (gayButton.wasJustReleased()) {
                parkAngle = 90;
            }

            if (imuReader.wasJustReleased()) {
                initialized = false;
                Walt.limelight.resetInitialized();
                Walt.localizer.setHeadingDegrees(180);
            }

            if (liftButton.wasJustReleased()) {
                lifting = !lifting;
            }


//            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
            telemetry.addData("Shooter: ", Walt.shooter.getTelemetry());
//            telemetry.addData("Shooter Timer: ", shootTimer.seconds());
            telemetry.addData("Limelight\n", Walt.limelight.getPositions());
            telemetry.addData("Heading: ", heading);
//            telemetry.addData("Is Limelight chilling: ", Gus.limelight.isAlive());
            telemetry.addData("Lim: ", Walt.limelight.getDistance());
            telemetry.addData("LoopRate: ", loopRateTracker.getLoopRateHz());
            telemetry.addData("Moving: ", moving);
            telemetry.addData("X: ", Walt.localizer.getPosX());
            telemetry.addData("Y: ", Walt.localizer.getPosY());
            telemetry.addData("Distance: ", Walt.limelight.getDistance());
            telemetry.addData("Dist constant: ", Walt.limelight.getDistConstant());
            telemetry.addData("Airtime: ", Walt.limelight.getAirTime());
            telemetry.addData("Turret Angle: ", Walt.shooter.getTurretAngle());
            telemetry.addData("Intake: ", Walt.intake.getTelemetry());
            telemetry.update();

        }



    }

    public void loadPose(File file) {
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String[] parts = reader.readLine().split(",");
            double x = Double.parseDouble(parts[0]);
            double y = Double.parseDouble(parts[1]);
            double heading = Double.parseDouble(parts[2]);
            Walt.limelight.initializeLocalizer(x, y, heading);
        } catch (IOException e) {
            return;
        }

    }
}