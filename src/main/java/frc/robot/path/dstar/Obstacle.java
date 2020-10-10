package frc.robot.path.dstar;

import java.util.List;

public class Obstacle {
	public final List<Node> vertexes;
	
	public Obstacle(List<Node> vertexes) {
		this.vertexes = List.copyOf(vertexes);
    }
}