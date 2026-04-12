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
import org.firstinspires.ftc.teamcode.Core.Gus;

import java.io.File;
import java.util.List;

@Config
@TeleOp
public class OneGamepadTeleop extends LinearOpMode {
    public static int multiplier = 1;

    public static boolean blue = false;
    boolean initialized = false;
    public static int shooterThreshold = 3;
    public static double antiShootPower = -0.12;
    public static double hoodK = 0.002;
    public static double xTranslation = 1.76, yTranslation = 1.3;
    public static double tP = 0.00002, tI = 0.000000, tD = 0;
    public static double tPF = 0.0000, tIF = 0.00000, tDF = 0;
    public static double P = 0.03, I=0.00, D = 0, F = 0.00325, S = 0.06;

    public static int visionDelay = 50;
    public static int pidTimerDelay =50;
    public static boolean moving = false;
    public static int targetVelocity = 0;
    public static double hoodPos = 0;

    int failsafeTargetVelocity = 163;
    double failsafeHoodPos = 0.38;
    LoopRateTracker loopRateTracker;

    protected Pose2D failSafePose = new Pose2D(DistanceUnit.INCH, 69.03, 83.0, DEGREES, 180);
    double parkHeading = 0;
    File file;
    boolean slowMo = false;


    @Override
    public void runOpMode() throws InterruptedException {
        file = AppUtil.getInstance().getSettingsFile("Headings.txt");
        loopRateTracker = new LoopRateTracker();
        boolean parking = false;
        boolean full = false;
        boolean reallyFull = false;
        boolean adjustingHood = false;
        boolean wasEmpty = false;
        boolean shooting = true;
        double magnitude, theta, driveTurn, x, y, heading, targetX = 0, targetY = 0, targetHeading = 0;
        boolean failsafe = false, initialized = false;
        double delay = 1;
        ElapsedTime shootTimer;
        ElapsedTime visionTimer;
        ElapsedTime pidTimer;
        pidTimer = new ElapsedTime();
        visionTimer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        ToggleButtonReader imuReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);

        ToggleButtonReader gateReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        ToggleButtonReader shootButton = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);


        double prevHeading = Double.parseDouble(ReadWriteFile.readFile(file));
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeInInit()) {
            Gus.initCamera(hardwareMap, blue);
            Gus.limelight.start();
        }

        waitForStart();
        Gus.init(hardwareMap, blue, true);
        Gus.intake.closeGate();

        while (opModeIsActive()) {
            moving = gamepad1.left_bumper;



            if (!initialized) {
                initialized = true;
                Gus.localizer.setHeadingDegrees(prevHeading + 90*multiplier);
            }
            loopRateTracker.updateLoopRate();
//            Gus.shooter.setPID(P, I, D, F, S);
//            Gus.limelight.setVelocityConstant(velocityConstant);
//            Gus.limelight.setShootConstant(shootTimeConstant);
//            Gus.shooter.setFullPowerThreshold(fullPowerThreshold);
//            Gus.shooter.setFineTurretPID(tPF, tIF, tDF);
//            Gus.shooter.setTurretKStatic(turretKStatic);
//            Gus.shooter.setTurretPID(tP, tI, tD);
//            Gus.shooter.setShooterThreshold(shooterThreshold);
//            Gus.shooter.setHoodAdjustmentConstant(hoodK);
//            Gus.limelight.setXYTranslation(xTranslation, yTranslation);


            Gus.localizer.update();
            if (Gus.intake.isOneBallIn() && visionTimer.milliseconds() > visionDelay) {
                Gus.limelight.trackAprilTag(Gus.localizer.getHeading()-180, Gus.shooter.getTurretAngle(), false);
                visionTimer.reset();
            }
            double distance = Gus.limelight.getDistance();
            if (distance > 0.5) {
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
                    Gus.shooter.setTargetVelocity(failsafeTargetVelocity);
                    Gus.shooter.setHood(failsafeHoodPos);
                }
            }
            else {
                Gus.shooter.setTargetVelocity(0);
                Gus.shooter.setTurretTargetPos(Shooter.angleToPosition(0));
            }

            if (pidTimer.milliseconds() > pidTimerDelay) {
                Gus.shooter.updateShooter();
                pidTimer.reset();
            }

            Gus.intake.updateIntake();

            if (Gus.intake.isFull() && !full) {
                full = true;
                gamepad1.rumble(200);
//                Gus.shooter.resetTurret();
            }

            if (gateReader.wasJustReleased()) {
                gamepad2.rumble(200);
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


            if (parking) {
                driveTurn = MotionPlanner.holdHeading(parkHeading);
            }
            else {
                driveTurn = -gamepad1.right_stick_x;
            }


            magnitude = Math.hypot(x, y);
            theta = Math.toDegrees(Math.atan2(y, x));
            heading = Gus.localizer.getHeading() - 180;
            theta = normalizeDegrees(theta - heading);

            Gus.drivetrain.drive(magnitude, theta, driveTurn, 1);




            if (gamepad1.left_trigger>0.3) {
                Gus.intake.rollerOut();
            }
            else if (gamepad1.left_bumper) {
                Gus.intake.rollerStop();
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
                    if (!slowMo) {
                        Gus.shooter.shoot();
                    }
                    else {
                        Gus.shooter.shootSlowMotion();
                    }
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
                Gus.limelight.setLocalizer(Gus.localizer.getHeading()-180, Gus.shooter.getTurretAngle());
            }




            gateReader.readValue();
            shootButton.readValue();
            imuReader.readValue();

            if (imuReader.wasJustReleased()) {
                Gus.limelight.resetInitialized();
                Gus.localizer.setHeadingDegrees(180);
            }


//            telemetry.addData("Turret: ", Gus.shooter.getTurretTelemetry());
            telemetry.addData("Shooter: ", Gus.shooter.getTelemetry());
//            telemetry.addData("Limelight\n", Gus.limelight.getPositions());
//            telemetry.addData("Localizer X: ", Gus.localizer.getPosX());
//            telemetry.addData("Localizer Y: ", Gus.localizer.getPosY());
//            telemetry.addData("X: ", Gus.localizer.getPosX());
//            telemetry.addData("Y: ", Gus.localizer.getPosY());
            telemetry.addData("Is Limelight chilling: ", Gus.limelight.isAlive());
            telemetry.addData("Heading: ", Gus.localizer.getHeading());
//            telemetry.addData("Intake: ", Gus.intake.getTelemetry());
//            telemetry.addData("Timer: ", shootTimer.seconds());
//            telemetry.addData("Moving: ", moving);
            telemetry.addData("LoopRate: ", loopRateTracker.getLoopRateHz());
//            telemetry.addData("ID: ", Gus.limelight.getFiducialID());
            telemetry.update();

        }



    }
}