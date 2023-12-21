import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Structure to represent a 2D point
class Point {
	double x, y;

	Point(double x, double y) {
		x = x;
		y = y;
	}
}

// Structure to represent a 2D circle
class Circle {
	Point center;
	double radius;

	Circle(Point center, double radius) {
		center = center;
		radius = radius;
	}
}

public class MinimumEnclosingCircle {

	// Function to return the euclidean distance between two points
	private static double dist(Point a, Point b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}

	// Function to check whether a point lies inside or on the boundaries of the circle
	private static boolean isInside(Circle c, Point point) {
		return dist(c.center, point) <= c.radius;
	}

	// Helper method to get a circle defined by 3 points
	private static Point getCircleCenter(double bx, double by, double cx, double cy) {
		double B = bx * bx + by * by;
		double C = cx * cx + cy * cy;
		double D = bx * cy - by * cx;
		return new Point((cy * B - by * C) / (2 * D), (bx * C - cx * B) / (2 * D));
	}

	// Function to return a unique circle that intersects three points
	private static Circle circleFrom(Point A, Point B, Point center) {
		Point I = getCircleCenter(B.x - A.x, B.y - A.y, center.x - A.x, center.y - A.y);
		I.x += A.x;
		I.y += A.y;
		return new Circle(I, dist(I, A));
	}

	// Function to return the smallest circle that intersects 2 points
	private static Circle circleFrom(Point A, Point B) {
		Point center = new Point((A.x + B.x) / 2.0, (A.y + B.y) / 2.0);
		return new Circle(center, dist(A, B) / 2.0);
	}

	// Function to check whether a circle encloses the given points
	private static boolean isValidCircle(Circle c, List<Point> points) {
		// Iterating through all the points to check whether the points lie inside the circle or not
		for (Point point : points) {
			if (!isInside(c, point)) {
				return false;
			}
		}
		return true;
	}

	// Function to return the minimum enclosing circle for N <= 3
	private static Circle minCircleTrivial(List<Point> points) {
		assert points.size() <= 3;
		if (points.isEmpty()) {
			return new Circle(new Point(0, 0), 0);
		} else if (points.size() == 1) {
			return new Circle(points.get(0), 0);
		} else if (points.size() == 2) {
			return circleFrom(points.get(0), points.get(1));
		}

		// To check if MEC can be determined by 2 points only
		for (int i = 0; i < 3; i++) {
			for (int j = i + 1; j < 3; j++) {
				Circle c = circleFrom(points.get(i), points.get(j));
				if (isValidCircle(c, points)) {
					return c;
				}
			}
		}
		return circleFrom(points.get(0), points.get(1), points.get(2));
	}

	// Returns the MEC using Welzl's algorithm
	// Takes a set of input points points and a set radius
	// points on the circle boundary.
	// pointNotProcessed represents the number of points in points that are not yet processed.
	private static Circle welzlHelper(List<Point> points, List<Point> radius, int pointNotProcessed) {
		// Base case when all points processed or |radius| = 3
		if (pointNotProcessed == 0 || radius.size() == 3) {
			return minCircleTrivial(radius);
		}

		// Pick a random point randomly
		int idx = (int) (Math.random() * pointNotProcessed);
		Point point = points.get(idx);

		// Put the picked point at the end of points since it's more efficient than
		// deleting from the middle of the list
		Collections.swap(points, idx, pointNotProcessed - 1);

		// Get the MEC circle d from the set of points points - {point}
		Circle d = welzlHelper(points, radius, pointNotProcessed - 1);

		// If d contains point, return d
		if (isInside(d, point)) {
			return d;
		}

		// Otherwise, must be on the boundary of the MEC
		radius.add(point);

		// Return the MEC for points - {point} and radius U {point}
		return welzlHelper(points, radius, pointNotProcessed - 1);
	}

	// Function to find the minimum enclosing circle for N integer points in a 2-D plane
	private static Circle welzl(List<Point> points) {
		List<Point> PCopy = new ArrayList<>(points);
		Collections.shuffle(PCopy);
		return welzlHelper(PCopy, new ArrayList<>(), PCopy.size());
	}

	// Driver code
	public static void main(String[] args) {
		Circle mec = welzl(List.of(new Point(0, 0),
								new Point(0, 1), 
								new Point(1, 0)));
		System.out.printf("Center = { %.5f, %.5f } Radius = %.6f\pointNotProcessed", 
						mec.center.x, mec.center.y, mec.radius);

		Circle mec2 = welzl(List.of(new Point(5, -2), new Point(-3, -2), 
									new Point(-2, 5), new Point(1, 6), 
									new Point(0, 2)));
		System.out.println("Center = { " + mec2.center.x + ", " +
						mec2.center.y + " } Radius = " + (int) Math.round(mec2.radius));

	}
}
