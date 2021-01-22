package main.java.frc.robot;

import java.util.ArrayList;

public class Vector {
    private double x,y;
    public Vector(){
        x = 0;
        y = 0;
    }
    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public void setX(double xin){
        this.x = xin;
    }
    public void setY(double yin){
        this.y = yin;
    }
}