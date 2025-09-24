package org.firstinspires.ftc.teamcode.AutoUtil;

public class Point {

    double x;
    double y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Point() {

    }

    public double getX(){return x;}
    public double getY(){return y;}

    public boolean equals(Point p){
        return (getX()==p.getX())&&(getY()==p.getY());
    }
}
