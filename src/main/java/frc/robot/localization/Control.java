package frc.robot.localization;


import java.awt.MouseInfo;
import java.util.ArrayList;
import java.util.Scanner;

import javax.swing.JFrame;

public class Control {

	public static void main(String[] args) {
		JFrame testFrame = new JFrame();
		testFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		testFrame.setSize(600, 600);
		MapDisplay display = new MapDisplay();
		testFrame.add(display);

		double angleRad, angle, distance;
		Scanner scan = new Scanner(System.in);
		System.out.println("Input Geometry by point (Format: x, y ) Input start point to end ");
		Shape room = new Shape(getPointInput(scan));
		System.out.println(room);

		System.out.println("Input Distance 1: ");
		distance = scan.nextDouble();
		System.out.println("Input camera angle 1 (relative) (0-360): ");
		angle = scan.nextDouble();
		angleRad = Math.toRadians(angle);
		

		System.out.println("Input Distance 2: ");
		distance = scan.nextDouble();
		System.out.println("Input camera angle 2 (relative) (0-360): ");
		angle = scan.nextDouble();
		angleRad = Math.toRadians(angle);
		testFrame.setVisible(true);
		for(;;) {
		angleRad = Math.toRadians(MouseInfo.getPointerInfo().getLocation().getY());
		Shape shiftedRoom1 = room.traslate(distance * Math.sin(angleRad), distance * Math.cos(angleRad));
		angleRad += Math.toRadians(23);
		Shape shiftedRoom2 = room.traslate(distance * Math.sin(angleRad), distance * Math.cos(angleRad));

		display.addMap(room);

		System.out.println(shiftedRoom1);
		System.out.println(shiftedRoom2);

		System.out.println("Input Shape : " + room);
		System.out.println("Possible location 1: " + shiftedRoom1);
		System.out.println("Possible location 2: " + shiftedRoom2);

		ArrayList<Line> shift1 = getLines(shiftedRoom1.getPoints());
		ArrayList<Line> shift2 = getLines(shiftedRoom2.getPoints());
		ArrayList<Point> outPoints = new ArrayList<Point>();
		for (Line line1 : shift1) {
			for (Line line2 : shift2) {
				Point t = Line.getLineIntersets(line1.getPoint1(), line1.getPoint2(), line2.getPoint1(), line2.getPoint2());
				if (t != null && room.contains(t)) {
					outPoints.add(t);
				}else if(t != null) {
					outPoints.add(t);
				}
			}
		}
		System.out.println("Location : " + outPoints);
		for (Point p : outPoints) {
			display.addInersect(p);
		}
		display.repaint();
		}
		
		

	}

	public static ArrayList<Point> getPointInput(Scanner scan) {
		ArrayList<Point> points = new ArrayList<Point>();
		boolean isBuilding = true;
		Point startPoint = null;
		while (isBuilding) {
			String[] data = scan.next().replaceAll(" ", "").split(",");
			Point point = new Point(Integer.parseInt(data[0]), Integer.parseInt(data[1]));
			if (startPoint != null) {
				if (point.equals(startPoint)) {
					isBuilding = false;
					continue;
				}
			} else {
				startPoint = point;
			}
			points.add(point);
		}
		return points;
	}

	public static ArrayList<Line> getLines(ArrayList<Point> points) {
		ArrayList<Line> out = new ArrayList<Line>();
		Point first = null;
		Point last = null;
		for (Point p : points) {
			if (last != null) {
				out.add(new Line(p, last));
				last = p;
			} else {
				last = p;
				first = p;
			}
		}
		out.add(new Line(first, last));
		return out;
	}

}