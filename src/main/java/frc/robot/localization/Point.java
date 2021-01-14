package frc.robot.localization;

public class Point {
	private double x;
	private double y;
	public Point(double x, double y){
		this.x = x;
		this.y = y;
	}
	public double getX() {
		return x;
	}
	public double getY() {
		return y;
	} 
	public boolean equals(Point obj) {
		return this.y == obj.getY() && this.x == obj.getX();
	}
	@Override
	public String toString() {
		return  "(" + x + "," + y + ")";
	}

}