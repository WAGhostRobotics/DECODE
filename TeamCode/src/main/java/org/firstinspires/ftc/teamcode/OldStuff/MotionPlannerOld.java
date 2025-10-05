package org.firstinspires.ftc.teamcode.OldStuff;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoUtil.Path;
import org.firstinspires.ftc.teamcode.AutoUtil.Point;
import org.firstinspires.ftc.teamcode.Components.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.RI3W.George;

@Config
public class MotionPlannerOld {

    private Path spline;
    private double targetHeading;
    double correction;

    private MecanumDrive drive;
    private PinpointLocalizer localizer;

    //    private PIDController translationalControl = new PIDController(0.022,0.001,0.03);
    public static PIDController translationalControl = new PIDController(0.01, 0.0001, 0.0);
    public static PIDController headingControl = new PIDController(0.008, 0.0001, 0);

    //    private PIDController translationalControlEnd = new PIDController(0.022,0.001,0.03);
//    public static PIDController translationalControlEnd = new PIDController(0.025,0.02,0.1);
    public static PIDController translationalControlEndX = new PIDController(0.04,0.003, 0); // 0.02 p 0.0015 i
    public static PIDController translationalControlEndY = new PIDController(0.069, 0.0065, 0);  // i term modified but might revert
    public static PIDController headingControlEnd = new PIDController(0.015, 0.0003, 0); // hope

    public boolean started;
    private int index;
    private double x;
    private double y;
    private double x_error;
    private double y_error;
    private final double permissible_translational_error = 2;
    private double translationalError;
    private double theta;
    private double magnitude;
    private double driveTurn;
    private double x_power;
    private double y_power;
    private double x_rotated;
    private double y_rotated;
    private double currentHeading;


    double y1;
    double y2;

    private double velocity;

    double lasty;
    double lasty1;
    double lastx;

    double currentY;
    double currentX;

    double radius;
    public final static double THE_HOLY_CONSTANT = 0.0006; //0.001

    public static double kStatic_X = 0.15; //.19
    public static double kStatic_Y = 0.34   ; //.19
    public static double kStatic_Turn = 0.11; //.19
    double ac;

    double numLoops;
    ElapsedTime loopTime;


    private double movementPower = 0.765;
    private double translational_error = 0.8;
    private double heading_error = 1;
    private final double endTrajThreshhold = 14;
    public static final double tIncrement = 0.05;


    boolean end = false;
    boolean setVelocity = false;
    public boolean toUpdate;

    private ElapsedTime ACtimer;

    private int estimatedStopping;

    Point target;
    Point derivative;

    double perpendicularError;


    HardwareMap hwMap;

    double voltage = 0;

    public MotionPlannerOld(MecanumDrive drive, PinpointLocalizer localizer, HardwareMap hwMap){
        translationalControlEndY.setIntegrationBounds(-1000000, 1000000);
        translationalControlEndX.setIntegrationBounds(-1000000, 1000000);
        headingControlEnd.setIntegrationBounds(-1000000, 1000000);
        translationalControl.setIntegrationBounds(-1000000, 1000000);
        headingControl.setIntegrationBounds(-1000000, 1000000);

        this.drive = drive;
        this.localizer = localizer;
        this.hwMap = hwMap;

    }

    public void reset() {
        ACtimer = new ElapsedTime();

        translationalControl.reset();
        headingControl.reset();

//        voltage = Pika.getVoltage();

        numLoops = 0;
        loopTime = new ElapsedTime();

        double length = spline.approximateLength();
        estimatedStopping = (int)(((length - endTrajThreshhold)/length)/tIncrement);

        index = 0;
    }

    public void startTrajectory(Path spline) {
        x_power = 0;
        y_power = 0;
        this.spline = spline;
        started = false;
        toUpdate = true;
        end = false;
        reset();
    }

//    public void startTrajectory(Bezier... splines) {
//        this.spline = new MergedBezier(splines);
//        reset();
//    }

    public Path getSpline(){
        return spline;
    }


    public String getTelemetry(){
        if(!toUpdate)
            return "";


        return "Index: " + index +
                "\n Targetheading: " + targetHeading +
                "\n Derivative: " + derivative.getX() + ", " + derivative.getY() + " " +
                "\nCorrection: " + correction +
                "\ny1: " + y1 +
                "\ny2: " + y2 +
                "\nCurrentX: " + currentX +
                "\nCurrentY: " + currentY +
                "\n Perpendicular error: " + (perpendicularError) +
                "\n End " + end +
                "\n Reached X: " + reachedXTarget() +
                "\n Reached Y: " + reachedYTarget() +
                "\n Reached Heading: " + reachedHeadingTarget() +
                "\n Reached Translational: " + reachedTranslationalTarget() +
                "\n Finished " + isFinished()+
                "\n Heading: " + currentHeading +
                "\n X: " + localizer.getPosX() +
                "\n Y: " + localizer.getPosY() +
                "\n TargetX: " + spline.getEndPoint().getX() +
                "\n TargetY: " + spline.getEndPoint().getY() +
                "\n X_error: " + x_error +
                "\n Y_error: " + y_error +
                "\n xPower: " + x_power +
                "\n yPower: " + y_power +
                "\n Theta: " + theta +
                "\n Magnitude: " + magnitude +
                "\n DriveTurn: " + driveTurn +
                "\nToUpdate: " + toUpdate +
                "\nStarted: " + started;
    }

    public double getPerpendicularError(){
        return perpendicularError;
    }


    public double getHeadingError(){
        double headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }

    public void update() {

        if (!toUpdate)
            return;
        updateACValues();

        if ((Double.isNaN(x_power) || Double.isNaN(y_power)) || Double.isNaN(magnitude) ||
                Double.isNaN(theta) || Double.isNaN(driveTurn)) {
            magnitude = x_power = y_power = correction = theta = driveTurn = 0;
        }

        x = localizer.getPosX();
        y = localizer.getPosY();
        currentHeading = normalizeDegrees(localizer.getHeading());
//        currentHeading = 0;

        while (index <= estimatedStopping && distance(spline.getCurvePoints()[index + 1], new Point(x, y)) <
                distance(spline.getCurvePoints()[index], new Point(x, y))) {
            index++;
        }

        target = spline.getCurvePoints()[index];
        targetHeading = spline.getCurveHeadings()[index];
        derivative = spline.getCurveDerivatives()[index];
        if(!isFinished()){
            started = true;

            if(index >=estimatedStopping){

                if(!end){
                    translationalControlEndX.reset();
                    translationalControlEndY.reset();
                    headingControlEnd.reset();
                }
                end = true;

                x_error = spline.getEndPoint().getX() - x;
                y_error = spline.getEndPoint().getY() - y;
                translationalError = Math.hypot(x_error, y_error);
                theta = normalizeDegrees(Math.toDegrees(Math.atan2(y_error, x_error)) - currentHeading);
                x_error = Math.cos(Math.toRadians(theta))*translationalError;
                y_error = Math.sin(Math.toRadians(theta))*translationalError;
                x_power = translationalControlEndX.calculate(0, x_error);
                y_power = translationalControlEndY.calculate(0, y_error);
                x_power = x_power + Math.signum(x_power)* kStatic_X;
                y_power = y_power + Math.signum(y_power)* kStatic_Y;

//                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
//                y_rotated =  -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));
//                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                // Calculate theta before adding KStatics because KStatics for x and y are not the same

                x_power = (!reachedXTarget()) ? (x_power): 0;
                y_power = (!reachedYTarget()) ? (y_power): 0;
//                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
//                y_rotated =  -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));


                magnitude = Math.hypot(x_power, y_power);
                driveTurn = headingControlEnd.calculate(0, getHeadingError());

                driveTurn =  (!reachedHeadingTarget()) ? (driveTurn + Math.signum(driveTurn) * kStatic_Turn) : 0;

//                drive.drive(magnitude, theta, driveTurn, movementPower, voltage);
                drive.drive(magnitude, theta, driveTurn, movementPower);
            } else {

                end = false;

                magnitude = 1;

                double vy = derivative.getY();
                double vx = derivative.getX();

                theta = Math.toDegrees(Math.atan2(vy, vx));

                double multiplier = 1;

                if(vx == 0){

                    perpendicularError = target.getX() - x;

                    if(vy<0){
                        multiplier = -1;
                    }

                }else{
                    double slope = vy/vx;
                    double yIntTarget = (target.getY() - (slope)*(target.getX()));
                    double yIntReal = (y - (slope)*x);

                    perpendicularError = Math.abs(yIntTarget-yIntReal)/Math.sqrt(1 + Math.pow(slope, 2));
                    perpendicularError = Math.signum(normalizeDegrees(theta-90)) * perpendicularError;

                    if(yIntTarget <= yIntReal){
                        multiplier = -1;
                    }

                }
                perpendicularError *= multiplier;
                correction = translationalControl.calculate(0, perpendicularError);
                if (Double.isNaN(correction))
                    correction = 0;
//                correction = 0;
                theta -= Math.toDegrees(Math.atan2(correction, magnitude));
                magnitude = Math.hypot(magnitude, correction);

                x_power = magnitude * Math.cos(Math.toRadians(theta));
                y_power = magnitude * Math.sin(Math.toRadians(theta));

                x_rotated = x_power * Math.cos(Math.toRadians(currentHeading)) + y_power * Math.sin(Math.toRadians(currentHeading));
                y_rotated = -x_power * Math.sin(Math.toRadians(currentHeading)) + y_power * Math.cos(Math.toRadians(currentHeading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                driveTurn = headingControl.calculate(0, getHeadingError());

//
//                if(!Double.isNaN(y1)&&!Double.isNaN(y2) && magnitude != 0){
//                    radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
//                    ac = Math.pow(velocity, 2)/radius;
////                    theta += Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, 1));
//                    magnitude *= Math.hypot(1, ac*THE_HOLY_CONSTANT);
//
//                }else{
//                    ac = 0;
//                }


//                drive.driveMax(magnitude, theta, driveTurn, movementPower, voltage);
                George.drivetrain.driveMax(magnitude, theta, driveTurn, 0.8);
            }

        }else{
            if(reachedXTarget()){
                translationalControlEndX.reset();
            }

            if(reachedYTarget()){
                translationalControlEndY.reset();
            }

            if(reachedHeadingTarget()){
                headingControlEnd.reset();
            }

            drive.drive(0, 0, 0, 0);
        }

        numLoops++;
    }

    public void updateACValues(){
        currentX = localizer.getEncoderY();
        currentY = localizer.getEncoderX();

        if((currentX-lastx) == 0){
            y1 = Double.NaN;
            y2 = Double.NaN;
        }else{
            y1 = (currentY-lasty)/(currentX-lastx);
            y2 = (y1-lasty1)/(currentX-lastx);
        }


        double ACtime = ACtimer.seconds();

        if (ACtime != 0) {
            velocity = Math.sqrt(
                    Math.pow(((currentX - lastx) / ACtime), 2) + Math.pow(((currentY - lasty) / ACtime), 2)
            );
        }
        if (!Double.isNaN(y1))
            lasty1 = y1;
        lastx = currentX;
        lasty = currentY;



        ACtimer.reset();


    }

    public boolean isFinished() {
        return  reachedTranslationalTarget() && reachedHeadingTarget();
    }

    private boolean reachedTranslationalTarget(){
        return (reachedXTarget() && reachedYTarget());
    }

    private boolean reachedXTarget(){
        return Math.abs(x_error)<= translational_error && end;
    }

    private boolean reachedYTarget(){
        return Math.abs(y_error)<= translational_error && end;
    }

    private boolean reachedHeadingTarget(){
        return (Math.abs(targetHeading - currentHeading)<= heading_error);
    }

    public void setPermissibleTranslationalError(double error) {
        this.translational_error = error;
    }

    public void setPermissibleHeadingError(double error) {
        this.heading_error = error;
    }

    private double distance(Point p1, Point p2){
        return Math.hypot(p1.getX()-p2.getX(), p1.getY()-p2.getY());
    }



    public void setTerms(double kX, double kY, double kT, double pX, double pY, double pT,
                         double iX, double iY, double iT, double dX, double dY, double dT,
                         double pTr, double iTr, double dTr) {
        kStatic_X = kX;
        kStatic_Y = kY;
        kStatic_Turn = kT;
        translationalControlEndX.setPID(pX, iX, dX);
        translationalControlEndY.setPID(pY, iY, dY);
        headingControl.setPID(pT, iT, dT);
        headingControlEnd.setPID(pT, iT, dT);
        translationalControl.setPID(pTr, iTr, dTr);

    }

    public void pause() {
        drive.drive(0, 0, 0, 0);
        toUpdate = false;
    }

    public void resume() {
        toUpdate = true;
    }

    public void setMovementPower(double power) {
        this.movementPower = power;
    }

    public boolean isEnd() {
        return end;
    }
}
