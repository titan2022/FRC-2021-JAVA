package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.generation.rmpflow.rmps.Cartesian2d;
import frc.robot.motion.generation.rmpflow.rmps.GoalAttractor;

public class AutoNavChallenge extends CommandBase {
    private Translation2d[] waypoints;
    private Translation2d waypoint;
    private int next = 0;
    public final RMPDrive drive;
    private Cartesian2d coord;
    private GoalAttractor attractor;
    private double threshold;
    private boolean done;

    public AutoNavChallenge(RMPDrive drive, double threshold, Translation2d... waypoints) {
        this.drive = drive;
        this.waypoints = waypoints;
        this.threshold = threshold;
        this.coord = new Cartesian2d("AutoNav 2d", drive.root, 0, 1);
        waypoint = drive.filter.getFilteredPose().getTranslation();
        this.attractor = new GoalAttractor("Waypoint", this.coord, toVector(waypoint), 10, 1, 10, 1, 2, 2, 0.005);
    }

    private SimpleMatrix toVector(Translation2d x) {
        return new SimpleMatrix(new double[][]{{x.getX()}, {x.getY()}});
    }

    @Override
    public void execute() {
        if(isFinished())
            return;
        if(waypoint.getDistance(drive.filter.getFilteredPose().getTranslation()) < threshold){
            if(next >= waypoints.length){
                done = true;
            }
            else{
                waypoint = waypoints[next];
                attractor.updateGoal(toVector(waypoints[next++]));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        coord.unlinkParent(coord.getParent());
    }
}
