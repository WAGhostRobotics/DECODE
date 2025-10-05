package org.firstinspires.ftc.teamcode.AutoUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.teamcode.AutoUtil.Bezier.tIncrement;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.DriveTrain.Drivetrain;

public class MotionPlanner {
    private Path spline;                    // Path to be followed (Can be Bezier, Merged or anything else)
    private double currentHeading, currentX, currentY, currentVelocity;
    private double targetHeading, targetX, targetY;
    private double xError, yError, headingError;
    private double xPower, yPower, magnitude, theta, driveTurn;
    private final PIDController translationalControlX = new PIDController(0.04,0.003, 0);
    private final PIDController translationalControlY =  new PIDController(0.069, 0.0065, 0);
    private final PIDController headingControl = new PIDController(0.015, 0.0003, 0);
    private final double kStaticX = 0.18;                 // Minimum power before robot moves in X direction
    private final double kStaticY = 0.3;
    private final double kStaticTurn = 0.2;
    private final double permissibleTranslationalError = 0.5, permissibleHeadingError = 1;          // Translational in inches, Heading in degrees
    private final double speedThresholdDistance = 15;       // 15 in before the end point, the robot will stop going full speed and start slowing down
    private double speedThresholdPoint;                     // Up until this "point" in the spline, robot goes full speed. Then slows down at the end
    private int index;                                      // Index of the Bezier that robot currently at

    private final PinpointLocalizer localizer;                // The motion planner WILL NOT update localizer. This needs to be done outside
    private Drivetrain drivetrain;
    private HardwareMap hardwareMap;
    private double voltage;
    private double movementPower;
    private boolean isEndOfSpline;
    private boolean toUpdate = true;                // Needed to pause motion planner at times


    public MotionPlanner(Drivetrain drivetrain, PinpointLocalizer localizer, HardwareMap hwMap) {
        translationalControlX.setIntegrationBounds(-10000000, 10000000);
        translationalControlY.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.hardwareMap = hwMap;
    }

    public void startFollowingPath(Path path) {
        toUpdate = true;
        this.spline = path;
        double length = path.approximateLength();
        speedThresholdPoint = (int)(((length - speedThresholdDistance)/length)/tIncrement);             // tIncrement comes from Bezier file
        index = 0;
        isEndOfSpline = false;
        reset();

    }

    private void reset() {
        translationalControlY.reset();
        translationalControlX.reset();
        headingControl.reset();
    }
    private void updateRobotValues() {
        currentHeading = localizer.getHeading();            // In degrees
        currentX = localizer.getPosX();
        currentY = localizer.getPosY();
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }
    public void getHeadingError(){
        headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }
    }


    public void update() {                  // This actually moves the robot
        updateRobotValues();                // Get current position and heading

        if (!toUpdate) {
            return;
        }

        // The loop below increments index until we reach the point closest to the robot's current (x, y)
        // We want to PID to the next point on the spline that is closest to us
        while (index <= speedThresholdPoint && distance(spline.getCurvePoints()[index + 1], new Point(currentX, currentY)) <
                distance(spline.getCurvePoints()[index], new Point(currentX, currentY))) {
            index++;
        }

        targetX = spline.getCurvePoints()[index].getX();
        targetY = spline.getCurvePoints()[index].getY();
        targetHeading = spline.getCurveHeadings()[index];

        xError = targetX - currentX;
        yError = targetY - currentY;
        getHeadingError();
        // These three lines are redundant if you look ahead in the code
        // All these values are recalculated before calling isFinished
        // to make sure the robot is still updating once it reaches the end of the spline
        // (Lil funky... if you don't get this, its fine just let it be)

        if (!isFinished()) {
            if (index >= speedThresholdPoint) {        // if nearing the end of the spline
                isEndOfSpline = true;
                targetX = spline.getEndPoint().getX();          // If at the end, we PID straight to the end point
                targetY = spline.getEndPoint().getY();
                targetHeading = spline.getCurveHeadings()[spline.getCurveHeadings().length-1];

                xError = targetX - currentX;
                yError = targetY - currentY;
                getHeadingError();

                double translationalError = Math.hypot(xError, yError);
                double theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);       // Theta relative to robot
                xError = Math.cos(Math.toRadians(theta))*translationalError;                // X and Y relative to robot
                yError = Math.sin(Math.toRadians(theta))*translationalError;
                xPower = translationalControlX.calculate(0, xError);
                yPower = translationalControlY.calculate(0, yError);
                xPower = xPower + Math.signum(xPower)* kStaticX;
                yPower = yPower + Math.signum(yPower)* kStaticY;
                xPower = (!reachedX()) ? (xPower): 0;
                yPower = (!reachedY()) ? (yPower): 0;

                magnitude = Math.hypot(xPower, yPower);
                driveTurn = headingControl.calculate(0, headingError);
                driveTurn =  (!reachedHeading()) ? (driveTurn + Math.signum(driveTurn) * kStaticTurn) : 0;

                drivetrain.drive(magnitude, theta, driveTurn, movementPower);
            }

            else {          // Speed mode (Mostly driven by direction of path) PID only comes into play when robot is off track
                magnitude = 1;
                Point derivative = spline.getCurveDerivatives()[index];
                double vy = derivative.getY();      // Y Magnitude
                double vx = derivative.getX();      // X Magnitude
                double perpendicularError;

                theta = Math.toDegrees(Math.atan2(vy, vx));     // Ideal direction of robot if it is on track

                double multiplier = 1;          // Basically to figure out if the robot is on the left or right of the target path

                if(vx == 0){

                    perpendicularError = targetX - currentX;

                    if(vy<0){
                        multiplier = -1;
                    }

                }else{
                    double slope = vy/vx;               // Slope of the path, if you will
                    double yIntTarget = (targetY - (slope)*(targetX));      // Y intercept of the target path
                    double yIntReal = (currentY - (slope)*currentX);                                  // Y intercept of the current path

                    perpendicularError = Math.abs(yIntTarget-yIntReal)/Math.sqrt(1 + Math.pow(slope, 2));       // Perpendicular Distance between the 2 parallel lines (Current Path vs Target Path)


                    perpendicularError = Math.signum(normalizeDegrees(theta-90)) * perpendicularError;      // Left or right of path ?
                    if(yIntTarget <= yIntReal){
                        multiplier = -1;
                    }

                }
                perpendicularError *= multiplier;
                double correction = translationalControlY.calculate(0, perpendicularError);         // Better to use Y PID control bc robot is usually facing forward towards the path
                theta -= Math.toDegrees(Math.atan2(correction, magnitude));
                magnitude = Math.hypot(magnitude, correction);

                xPower = magnitude * Math.cos(Math.toRadians(theta));
                yPower = magnitude * Math.sin(Math.toRadians(theta));

                double x_rotated = xPower * Math.cos(Math.toRadians(currentHeading)) + yPower * Math.sin(Math.toRadians(currentHeading));
                double y_rotated = -xPower * Math.sin(Math.toRadians(currentHeading)) + yPower * Math.cos(Math.toRadians(currentHeading));

                magnitude = Math.hypot(x_rotated, y_rotated);
                theta = Math.toDegrees(Math.atan2(y_rotated, x_rotated));
                getHeadingError();
                driveTurn = headingControl.calculate(0, headingError);
                drivetrain.driveMax(magnitude, theta, driveTurn, movementPower, voltage);


            }
        }
        else{
            if(reachedX()){
                translationalControlX.reset();
            }

            if(reachedY()){
                translationalControlY.reset();
            }

            if(reachedHeading()){
                headingControl.reset();
            }

            drivetrain.drive(0, 0, 0, 0);
        }

    }



    private boolean reachedX() {
        return Math.abs(xError) < permissibleTranslationalError && isEndOfSpline;
    }

    private boolean reachedY() {
        return Math.abs(yError) < permissibleTranslationalError && isEndOfSpline;
    }

    private boolean reachedHeading() {
        return Math.abs(headingError) < permissibleHeadingError && isEndOfSpline;
    }

    public boolean isFinished() {
        return reachedX() && reachedY() && reachedHeading();
    }

    public void setXPID(double p, double i, double d) {
        translationalControlX.setPID(p, i, d);
    }

    public void setYPID(double p, double i, double d) {
        translationalControlY.setPID(p, i, d);
    }

    public void setHeadingPID(double p, double i, double d) {
        headingControl.setPID(p, i, d);
    }
    public void setMovementPower(double power) {
        this.movementPower = power;
    }
    private double distance(Point p1, Point p2){                // Dist b/w two points (pythagorean)
        return Math.hypot(p1.getX()-p2.getX(), p1.getY()-p2.getY());
    }

    public void pause() {
        drivetrain.drive(0, 0, 0, 0);
        toUpdate = false;
    }

    public void resume() {
        toUpdate = true;
    }

    public String getTelemetry() {
        return "Updating: " + toUpdate +
                "\nX Error: " + xError +
                "\nY Error: " + yError +
                "\nX Power: " + xPower +
                "\nY Power: " + yPower +
                "\nCurrent X: " + currentX +
                "\nTarget X: " + targetX +
                "\nCurrent Y: " + currentY +
                "\nTarget Y: " + targetY +
                "\nIndex: " + index;
    }

}
