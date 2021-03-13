package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.Point;
import frc.robot.motion.generation.rmpflow.rmps.Cartesian2d;
import frc.robot.motion.generation.rmpflow.rmps.GoalAttractor;

public class AutoNavChallenge extends CommandBase {
    private Point[] waypoints;
    private int next = 0;
    public final RMPDrive drive;
    private Cartesian2d coord;
    private GoalAttractor attractor;

    public AutoNavChallenge(RMPDrive drive, Point... waypoints) {
        this.drive = drive;
        this.waypoints = waypoints;
        this.coord = new Cartesian2d("AutoNav 2d", drive.root, 0, 1);
        this.attractor = new GoalAttractor("Waypoint", this.coord, toVector(new Point(0, 0)), 10, 1, 10, 1, 2, 2, 0.005);
    }

    private SimpleMatrix toVector(Point x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}});
    }
}
