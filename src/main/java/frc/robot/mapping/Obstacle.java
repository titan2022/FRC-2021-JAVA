package frc.robot.mapping;

import java.util.Comparator;
import java.util.Iterator;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * An object included in a map of the field.
 * 
 * <p>Obstacles must also implement {@link Comparator<Point>} to induce an
 * ordering over points. For points of some equal distance r units away from
 * the edge of this obstacle, this ordering should correspond to the order the
 * points are encountered by a point moving around the effective perimeter of
 * this obstacle with an obstacle growth radius of r units. The implementation
 * may decide where along that perimeter the point begins and in which direction
 * it moves. The ordering may, but is not rquired to exist for points of
 * different distances from this obstacle. If it does exist for such points, it
 * is not required to (and most likely will not) comply with Object.equals() for
 * such pairs of points. However, the ordering must comply with Object.equals()
 * for pairs of points which are the same distance from the edge of this
 * obstacle.
 * 
 * @see ObstacleMap
 */
public interface Obstacle extends Comparator<Point> {
    /**
     * Determines whether a path is obstructed by this obstacle.
     * 
     * @param path  The path to test.
     * @param radius  The obstacle growth radius to use. That is, the minimum
     *  distance the path must stay away from this obstacle to be considered
     *  clear of this obstacle.
     * @return True if the path is clear of this obstacle, or false, otherwise.
     */
    default public boolean isClear(Path path, double radius) {
        throw new UnsupportedOperationException();
    }
    /**
     * Determines whether a path is obstructed by this obstacle.
     * 
     * <p>This method is equivalent to {@link #isClear(Path, double)} with a
     * radius of 0. This method may be more efficient for this special case,
     * or it may simply delegate to the more general version of this method.
     * 
     * @param path  The path to test.
     * @return True if the path is clear of this obstacle, or false, otherwise.
     */
    default public boolean isClear(Path path) {
        return isClear(path, 0);
    }

    /**
     * Gets the endpoints of this obstacle, as seen from a given point.
     * 
     * <p>These "endpoints" are the points of tangency with this obstacle of
     * lines draw through the given point.
     * 
     * @param source  The point the endpoints are seen from.
     * @param radius  The obstacle growth radius to use. The returned points
     *  will all be exactly {@code radius} units away from this obstacle.
     * @return An iterable over the endpoints of this obstacle, as seen from the
     *  given point.
     */
    public Iterable<Point> getEndpoints(Point source, double radius);
    /**
     * Gets the endpoints of this obstacle, as seen from a given point.
     * 
     * <p>This method is equivalent to {@link #getEndpoints(Point, radius)} with
     * a radius of 0. This method may be more efficient for this special case,
     * or it may simply delegate to the more general version of this method.
     * 
     * @param source  The point the endpoints are seen from.
     * @return An iterable over the endpoints of this obstacle, as seen from the
     *  given point.
     */
    default public Iterable<Point> getEndpoints(Point source) {
        return getEndpoints(source, 0);
    }

    /**
     * Gets the tangent lines between this obstacle and another.
     * 
     * <p>The default implementation of this method calls the
     * {@link #getTangents(Obstacle, radius)} method of the other obstacle with
     * this obstacle and the same radius as arguments. If this results in a
     * StackOverflowError (for instance, if both obstacles are using this
     * default implementation), an UnsupportedOperationException will be thrown.
     * Implementations of the Obstacle interface are strongly encouraged to
     * implement this method or overload it for as many cases as possible.
     * 
     * @param other  The other obstacle to get tangent lines between.
     * @param radius  The obstacle growth radius to use.
     * @return An iterable over the tangent lines between this obstacle and the
     *  other specified obstacle. Each tangent is represented as a LinearSegment
     *  from the point of tangency with this obstacle to the point of tangency
     *  with the other obstacle.
     */
    default public Iterable<LinearSegment> getTangents(Obstacle other, double radius) {
        try{
            Obstacle self = this;
            return new Iterable<LinearSegment>(){
                private Iterable<LinearSegment> original = other.getTangents(self, radius);

                @Override
                public Iterator<LinearSegment> iterator() {
                    return new Iterator<LinearSegment>(){
                        private Iterator<LinearSegment> it = original.iterator();

                        @Override
                        public boolean hasNext() {
                            return it.hasNext();
                        }

                        @Override
                        public LinearSegment next() {
                            return it.next().reverse();
                        }
                    };
                }
            };
        }
        catch(StackOverflowError err){
            throw new UnsupportedOperationException(err);
        }
    }
    /**
     * Gets the tangent lines between this obstacle and another.
     * 
     * <p>This method is equivalent to {@link #getTangents(Obstacle, double)}
     * with a radius of 0.
     * 
     * @param other  The other obstacle to get tangent lines between.
     * @param radius  The obstacle growth radius to use.
     * @return An iterable over the tangent lines between this obstacle and the
     *  other specified obstacle. Each tangent is represented as a LinearSegment
     *  from the point of tangency with this obstacle to the point of tangency
     *  with the other obstacle.
     */
    default public Iterable<LinearSegment> getTangents(Obstacle other) {
        return getTangents(other, 0);
    }

    /**
     * Applies a translation to this obstacle.
     * 
     * @param offset  The translation to apply.
     * @return The translated obstacle.
     */
    public Obstacle translateBy(Translation2d offset);

    /**
     * Applies a rotation to this obstacle.
     * 
     * @param rotation  The rotation to apply.
     * @return The rotated obstacle.
     */
    public Obstacle rotateBy(Rotation2d rotation);

    /**
     * Determines whether a point is an endpoint of this obstacle.
     * 
     * @param endpoint  The potential endpoint of this obstacle.
     * @param other  The point this obstacle is being viewed from to determine
     *  endpoints.
     * @param radius  The obstacle growth radius to use. The potential endpoint
     *  should be exactly this many units away from the edge of this obstacle.
     * @return True, if the potential endpoint is an endpoint of this obstacle
     *  as viewed from the given point with the specified obstacle growth
     *  radius, or false, otherwise.
     */
    default public boolean isEndpoint(Point endpoint, Point other, double radius) {
        for(Point canidate : getEndpoints(other, radius))
            if(canidate.equals(endpoint)) return true;
        return false;
    }
    /**
     * Determines whether a point is an endpoint of this obstacle.
     * 
     * <p>This method is equivalent to {@link #isEndpoint(Point, Point, double)}
     * with an obstacle growth radius of 0.
     * 
     * @param endpoint  The potential endpoint of this obstacle.
     * @param other  The point this obstacle is being viewed from to determine
     *  endpoints.
     * @return True, if the potential endpoint is an endpoint of this obstacle
     *  as viewed from the given point, or false, otherwise.
     */
    default public boolean isEndpoint(Point endpoint, Point other) {
        return isEndpoint(endpoint, other, 0);
    }

    /**
     * Gets the length of the perimeter of this obstacle.
     * 
     * @param radius  The obstacle growth radius to use.
     * @return The length of the perimeter of this obstacle after obstacle
     *  growth with the specified radius.
     */
    public double getPerimeter(double radius);
    /**
     * Gets the length of the perimeter of this obstacle.
     * 
     * <p>This method is equivalent to {@link #getPerimeter(double)} with a
     * radius of 0.
     * 
     * @return The perimeter of this obstacle without obstacle growth.
     */
    default public double getPerimeter() {
        return getPerimeter(0);
    }
}