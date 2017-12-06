package NonRigid2D;

import java.util.List;
import ij.process.ImageProcessor;

public class PointsOrientation {

	private List<double[]> points;
	private double[] centroid;
	private double orientation;

	private int length = 50;

	public PointsOrientation(List<double[]> points) {

		this.points = points;
		centroid = PointCollection.calculateCentroid(points);

		calculateOrientation();

	}

	private void calculateOrientation() {

		orientation = Math.atan2((2 * centralMoment(1, 1)), centralMoment(2, 0) - centralMoment(0, 2)) / 2.0;

	}

	private double centralMoment(int p, int q) {

		double moment = 0.0;

		for (double[] point : points) {
			moment += Math.pow((point[0] - centroid[0]), p) * Math.pow((point[1] - centroid[1]), q);
		}

		return moment;

	}

	public List<double[]> allignAxis() {
		
		points = Matrix.translate(points, -centroid[0], -centroid[1]);
		points = Matrix.rotate(points, -orientation);
		points = Matrix.translate(points, centroid[0], centroid[1]);
		
		return points;
	}

	public void drawAxis(ImageProcessor cp) {

		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];
		int x2 = (int) (x1 + length * Math.cos(orientation));
		int y2 = (int) (y1 + length * Math.sin(orientation));

		cp.drawLine(x1, y1, x2, y2);
	}

	public double getOrientation() {
		return orientation;
	}

	public double[] getCentroid() {
		return centroid;
	}

}
