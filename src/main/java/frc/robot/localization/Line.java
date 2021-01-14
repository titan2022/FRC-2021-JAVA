package frc.robot.localization;

public class Line {
	private Point p1;
	private Point p2;
	public Line(Point p1,Point p2) {
		this.p1 = p1;
		this.p2 = p2;
	}
	public Point getPoint1(){
		return p1;
	}
	public Point getPoint2(){
		return p2;
	}
	public static Point getLineIntersets(Point p1, Point p2, Point q1, Point q2){
		double x1,x2,x3,x4,y1,y2,y3,y4,p;
		x1=p1.getX();
		x2=p2.getX();
		x3=q1.getX();
		x4=q2.getX();
		y1=p1.getY();
		y2=p2.getY();
		y3=q1.getY();
		y4=q2.getY();


		 double bot = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
		  if (bot == 0.0) { // Lines are parallel.
		     return null;
		  }
		  double m1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/bot;
		  double m2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/bot;
		    if (m1 >= 0.0f && m1 <= 1.0f && m2 >= 0.0f && m2 <= 1.0f) {
		    	return new Point( (x1 + m1*(x2 - x1)), (y1 + m1*(y2 - y1)));
		    }

		  return null;
		  }
	}