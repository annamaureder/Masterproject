package NonRigid2D;

import java.awt.geom.Point2D;
import java.util.List;

public class ClusterPoint1 extends Point2D {

	private double[] coordinates = new double[2];

	public ClusterPoint1(double x, double y) {
		coordinates[0] = x;
		coordinates[1] = y;
	}

	public ClusterPoint1(double[] points) {
		coordinates = points;
	}

	public double[] subtract(ClusterPoint1 p) {
		double[] result = new double[p.coordinates.length];
		for (int i = 0; i < p.coordinates.length; i++) {
			result[i] = this.coordinates[i] - p.coordinates[i];
		}
		return result;
	}

	public double[] getCoordinates() {
		return coordinates;
	}

	public void setCoordinates(double[] coordinates) {
		this.coordinates = coordinates;
	}

	public double getX() {
		return coordinates[0];
	}

	public double getY() {
		return coordinates[1];
	}

	public void setLocation(double x, double y) {
		coordinates[0] = x;
		coordinates[1] = y;

	}
}
