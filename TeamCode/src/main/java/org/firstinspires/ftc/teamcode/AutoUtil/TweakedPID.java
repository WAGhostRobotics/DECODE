package org.firstinspires.ftc.teamcode.AutoUtil;

public class TweakedPID {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral;
    private double maxIntegral;
    private double errorVal_p;
    private double errorVal_v;
    private double totalError;
    private double prevErrorVal;
    private double errorTolerance_p;
    private double errorTolerance_v;
    private long lastTimeStamp;
    private double period;

    public TweakedPID(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, (double)0.0F, (double)0.0F);
    }

    public TweakedPID(double kp, double ki, double kd) {
        this(kp, ki, kd, 0, (double)0.0F, (double)0.0F);
    }

    public TweakedPID(double kp, double ki, double kd, double kf, double sp, double pv) {
        this.errorTolerance_p = 0.05;
        this.errorTolerance_v = Double.POSITIVE_INFINITY;
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kF = kf;
        this.setPoint = sp;
        this.measuredValue = pv;
        this.minIntegral = (double)-1.0F;
        this.maxIntegral = (double)1.0F;
        this.lastTimeStamp = 0;
        this.period = (double)0.0F;
        this.errorVal_p = this.setPoint - this.measuredValue;
        this.reset();
    }

    public void reset() {
        this.totalError = (double)0.0F;
        this.prevErrorVal = (double)0.0F;
        this.lastTimeStamp = 0;
    }

    public void setTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.errorTolerance_p = positionTolerance;
        this.errorTolerance_v = velocityTolerance;
    }

    public double getSetPoint() {
        return this.setPoint;
    }

    public void setSetPoint(double sp) {
        this.setPoint = sp;
        this.errorVal_p = this.setPoint - this.measuredValue;
        this.errorVal_v = (this.errorVal_p - this.prevErrorVal) / this.period;
    }

    public boolean atSetPoint() {
        return Math.abs(this.errorVal_p) < this.errorTolerance_p && Math.abs(this.errorVal_v) < this.errorTolerance_v;
    }

    public double[] getCoefficients() {
        return new double[]{this.kP, this.kI, this.kD, this.kF};
    }

    public double getPositionError() {
        return this.errorVal_p;
    }

    public double[] getTolerance() {
        return new double[]{this.errorTolerance_p, this.errorTolerance_v};
    }

    public double getVelocityError() {
        return this.errorVal_v;
    }

    public double calculate() {
        return this.calculate(this.measuredValue);
    }

    public double calculate(double pv, double sp) {
        this.setSetPoint(sp);
        return this.calculate(pv);
    }

    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        long currentTimeStamp = System.nanoTime()/ (long) 1e9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (measuredValue == pv) {
            errorVal_p = setPoint - measuredValue;
        } else {
            errorVal_p = setPoint - pv;
            measuredValue = pv;
        }

        if ((period) > 0.0) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // returns u(t)
        return kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;

    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kF = kf;
    }

    public void setPID(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        this.minIntegral = integralMin;
        this.maxIntegral = integralMax;
    }

    public void clearTotalError() {
        this.totalError = (double)0.0F;
    }

    public void setP(double kp) {
        this.kP = kp;
    }

    public void setI(double ki) {
        this.kI = ki;
    }

    public void setD(double kd) {
        this.kD = kd;
    }

    public void setF(double kf) {
        this.kF = kf;
    }

    public double getP() {
        return this.kP;
    }

    public double getI() {
        return this.kI;
    }

    public double getD() {
        return this.kD;
    }

    public double getF() {
        return this.kF;
    }

    public double getPeriod() {
        return this.period;
    }
}
