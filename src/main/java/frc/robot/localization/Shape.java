package frc.robot.localization;
import java.util.ArrayList;


public class Shape {
	ArrayList<Point> vertices = new ArrayList<Point>(); 
	public Shape(ArrayList<Point> points ) {
		for(Point point: points){
			vertices.add(point);
		}
	}
	public Shape traslate(double x, double y){
		ArrayList<Point> newVertices = new ArrayList<Point>();
		for(Point point: vertices){
			newVertices.add(new Point (point.getX() + x,point.getY() + y));
		}
		return new Shape(newVertices);
		
	}
	public double maxX(){
		double maxX= Double.MIN_VALUE;
		for(Point point: vertices){
			maxX=(maxX<point.getX())?point.getX():maxX;
		}
		return maxX;
	}
	public double minX(){
		double minX= Double.MAX_VALUE;
		for(Point point: vertices){
			minX=(minX>point.getX())?point.getX():minX;
		}
		return minX;
	}
	public double maxY(){
		double maxY= Double.MIN_VALUE;
		for(Point point: vertices){
			maxY=(maxY<point.getY())?point.getX():maxY;
		}
		return maxY;
	}
	public double minY(){
		double minY= Double.MAX_VALUE;
		for(Point point: vertices){
			minY=(minY>point.getY())?point.getX():minY;
		}
		return minY;
	}
	public ArrayList<Point> otherPoints(Point point){
		ArrayList<Point> otherPoints= new ArrayList<Point>();
		Point input=point;
		for(int x=0;x<vertices.size();x++){
			if(vertices.get(x).getY()>input.getY()&&
					vertices.get((x+1)%vertices.size()).getY()<input.getY()){
				otherPoints.add(vertices.get(x));
				otherPoints.add(vertices.get((x+1)%vertices.size()));
			}	
		}
		return otherPoints;
	}
	public double intersectingX(Point p1, Point p2, Point check){
		Point top = (p1.getY()>p2.getY())?p1:p2;
		Point bot = (p1.getY()>p2.getY())?p2:p1;
		if(p1.getX()==p2.getX()&&check.getY()<top.getY()&&check.getY()>bot.getY())
			return p1.getX();
		if(p1.getY()==p2.getY()&&p1.getY()==check.getY())
			return check.getX();
		double percent = (top.getY()-check.getY())/(top.getY()-bot.getY());
		double x = percent*(Math.abs(top.getX()-bot.getX()))+Math.min(top.getX(), bot.getX());
		return x;
	}
	
	public int rightIntersects(Point point){
		ArrayList<Point> otherPoint= otherPoints(point);
		int count=0;
		double y= point.getY();
		double x = point.getX();
		for(int b=0;b<otherPoint.size();b+=2){
			if(x<=intersectingX(otherPoint.get(b),otherPoint.get(b+1),point)){
				count++;
			}
			if(otherPoint.get(b).getX()==otherPoint.get(b+1).getX()
					&&x>(otherPoint.get(b).getX())){
				count++;
			}
		}
		
		return count;
	}
	public boolean contains(Point point){
		double y = point.getY();
		double x = point.getX();
		if(x>maxX()||x<minX()||y>maxY()||y<minY()||rightIntersects(point)%2==0){
//			System.out.println("rightintersects"+rightIntersects(point));
			return false;
		}
		return true;
	}
	public ArrayList<Point> getPoints(){
		return vertices;
	}
	@Override
	public String toString() {
		return vertices.toString();
	}
}
