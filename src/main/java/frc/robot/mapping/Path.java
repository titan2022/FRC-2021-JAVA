package frc.robot.mapping;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface Path {
    public double getLength();
	
    public Point getPos(double distance);

	default public Point getStart() {
        return getPos(0);
    }

	default public Point getEnd() {
        return getPos(getLength());
    }
	
	public Rotation2d getRotation(double distance);
	
    default public Point getVelocity(double distance, double speed) {
        return getVelocity(distance).times(speed);
    }
    default public Point getVelocity(double distance) {
        return new Point(1, getRotation(distance));
    }
	
	default public Rotation2d getAngularVelocity(double distance, double speed) {
        return getAngularVelocity(distance).times(speed);
    }
    public Rotation2d getAngularVelocity(double distance);
	
	default public Point getAcceleration(double distance, double speed, double acceleration) {
        return getAcceleration(distance, speed)
                .plus(new Point(acceleration, getRotation(distance)));
    }
    default public Point getAcceleration(double distance, double speed) {
        return getAcceleration(distance).times(speed);
    }
    default public Point getAcceleration(double distance){
        return new Point(getAngularVelocity(distance).getRadians(),
                getRotation(distance).plus(new Rotation2d(Math.PI / 2)));
    }
	
	public Path translateBy(Translation2d offset);
	
    public Path rotateBy(Rotation2d angle);
    
    public Path reverse();
	
    public double getDistance(Point from);
    
	default public double getDistance(Path other) {
        try{
            return other.getDistance(this);
        }
        catch(StackOverflowError err){
            throw new UnsupportedOperationException(err);
        }
    }

	default public boolean intersects(Path other) {
        return getDistance(other) == 0;
    }
}