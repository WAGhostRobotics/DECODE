package org.firstinspires.ftc.teamcode.AutoUtil;

public interface Path {
    public Point getCurvePoints(int i);
    public Point getCurveDerivatives(int i);
    public double getCurveHeadings(int i);
    public double approximateLength();

    public double getFinalHeading();

    public Point getEndPoint();

}
