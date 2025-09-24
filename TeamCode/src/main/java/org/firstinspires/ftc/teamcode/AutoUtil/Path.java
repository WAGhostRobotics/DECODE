package org.firstinspires.ftc.teamcode.AutoUtil;

public interface Path {
    public Point[] getCurvePoints();
    public Point[] getCurveDerivatives();
    public double[] getCurveHeadings();
    public double approximateLength();

    public Point getEndPoint();

}
