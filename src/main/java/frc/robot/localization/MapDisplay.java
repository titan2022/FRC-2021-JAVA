package frc.robot.localization;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;

import javax.swing.JComponent;

public class MapDisplay extends JComponent {
	/**
	 * 
	 */

	private static int MAP_SCALE = 32;
	private static int MAP_OFFSET_Y = 70;
	private static int MAP_OFFSET_X = 70;
	private static int Map_PONT_SIZE = 2;
	private ArrayList<Shape> offsetMaps = new ArrayList<Shape>();
	private Shape map;
	private ArrayList<Point> intersects = new ArrayList<Point>();

	@Override
	public synchronized void paint(Graphics g) {
		Graphics2D g2D = (Graphics2D) g;
//		g2D.setStroke(new BasicStroke(1));
		g2D.setColor(Color.BLACK);
		for (Shape Shape : offsetMaps) {
			renderShape(g2D, Shape);
		}
//		g2D.setStroke(new BasicStroke(4));
		g2D.setColor(Color.RED);

		renderShape(g2D, map);
//		g2D.setStroke(new BasicStroke(1));
		for (Point p : intersects) {
			g.setColor(Color.RED);
			g.drawOval((int) ajustX(p.getX()) - Map_PONT_SIZE / 2, (int) ajustY(p.getY()) - Map_PONT_SIZE / 2, Map_PONT_SIZE, Map_PONT_SIZE);
			g.setColor(Color.GREEN);
			g.fillOval((int) ajustX(p.getX()) - Map_PONT_SIZE / 2, (int) ajustY(p.getY()) - Map_PONT_SIZE / 2, Map_PONT_SIZE, Map_PONT_SIZE);
		}
		super.paint(g);
	}

	public synchronized void addOffsetMap(Shape shape) {
		offsetMaps.add(shape);
	}

	public synchronized void addMap(Shape shape) {
		map = shape;
	}

	public synchronized void addInersect(Point p) {
		intersects.add(p);

	}

	private void renderShape(Graphics2D g, Shape shape) {
		if (shape == null) {
			return;
		}
		Point first = null;
		Point last = null;
		for (Point p : shape.vertices) {
			if (last != null) {
				g.draw(new Line2D.Double(ajustX(p.getX()), ajustY(p.getY()), ajustX(last.getX()), ajustY(last.getY())));
				last = p;
			} else {
				last = p;
				first = p;
			}
		}
		g.draw(new Line2D.Double(ajustX(first.getX()), ajustY(first.getY()), ajustX(last.getX()), ajustY(last.getY())));

	}

	private double ajustY(double y) {
		return MAP_OFFSET_Y + (y * MAP_SCALE);
	}

	private double ajustX(double x) {
		return MAP_OFFSET_X + (x * MAP_SCALE);
	}

}