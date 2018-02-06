package NonRigid2D;

import java.util.List;
import java.util.ArrayList;

import ij.IJ;
import ij.process.ImageProcessor;

/**
 * This class collects all points from an ImageProcessor into a cluster -
 * calculation of centroid - calculation of orientation - calculation of the
 * principal and secondary axes - method to horizontally align the object -
 * method to draw the principal and secondary axes
 * 
 * @author WB
 *
 */

public class Cluster {

	private List<double[]> points = new ArrayList<double[]>();
	private double distanceThreshold = 10;

	private double[] centroid;
	private double orientation;
	private int axisLength = 100;
	private int principalLength;
	private int secondaryLength;
	private boolean removeNoise;

	public Cluster(ImageProcessor ip, boolean removeNoise) {
		this.removeNoise = removeNoise;
		this.points = getPoints(ip);
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	public Cluster(List<double[]> p) {
		this.points = p;
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	public Cluster(Cluster c) {
		this.points = c.points;
		this.centroid = c.centroid;
		this.orientation = c.orientation;
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	private List<double[]> getPoints(ImageProcessor ip) {

		int w = ip.getWidth();
		int h = ip.getHeight();

		List<double[]> pntlist = new ArrayList<double[]>();

		for (int v = 0; v < h; v++) {
			for (int u = 0; u < w; u++) {
				int p = ip.getPixel(u, v);

				if (p < -1) {
					pntlist.add(new double[] { u, v });
				}
			}
		}

		if (removeNoise) {
			IJ.log("Remove Noise");
			pntlist = biggestCluster(pntlist);
		}

		return pntlist;

	}

	private List<double[]> biggestCluster(List<double[]> allPoints) {

		List<double[]> current = new ArrayList<double[]>();
		List<double[]> maximum = new ArrayList<double[]>();

		while (!allPoints.isEmpty()) {

			current.add(allPoints.get(0));

			for (int c = 0; c < current.size(); c++) {
				allPoints.removeAll(current);
				for (int i = 0; i < allPoints.size(); i++) {
					if (distance(current.get(c), allPoints.get(i)) < distanceThreshold) {
						current.add(allPoints.get(i));
					}
				}
			}

			allPoints.removeAll(current);

			if (current.size() > maximum.size()) {
				maximum = current;
			}

			current = new ArrayList<double[]>();

		}

		return maximum;

	}

	private double distance(double[] current, double[] point) {

		double x = Math.abs(current[0] - point[0]);
		double y = Math.abs(current[1] - point[1]);

		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	private double[] calculateCentroid(List<double[]> points) {

		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i)[0];
			avgY += points.get(i)[1];
		}

		return new double[] { avgX / points.size(), avgY / points.size() };
	}

	private double calculateOrientation() {

		return Math.atan2((2 * centralMoment(1, 1)), centralMoment(2, 0) - centralMoment(0, 2)) / 2.0;
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

		orientation = 0;

		return points;
	}

	public void drawPrincipalAxis(ImageProcessor cp) {

		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];

		int x_test = (int) centroid[0] - 30;

		int x2 = (int) (x1 + axisLength / 2 * Math.cos(orientation));
		int y2 = (int) (y1 + axisLength / 2 * Math.sin(orientation));
		int x2_ = (int) (x1 - (axisLength / 2) * Math.cos(orientation));
		int y2_ = (int) (y1 - (axisLength / 2) * Math.sin(orientation));

		int y = (int) (Math.sin(orientation) * (x_test - x1) + y1);

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
	}

	public void drawSecondaryAxis(ImageProcessor cp) {

		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];

		int x2 = (int) (x1 + (axisLength / 4) * Math.cos(orientation + 1.5707963268));
		int y2 = (int) (y1 + (axisLength / 4) * Math.sin(orientation + 1.5707963268));
		int x2_ = (int) (x1 - (axisLength / 4) * Math.cos(orientation + 1.5707963268));
		int y2_ = (int) (y1 - (axisLength / 4) * Math.sin(orientation + 1.5707963268));

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);

	}

	public int getPrincipalRadius() {

		// allignAxis();

		int minX = Integer.MAX_VALUE;
		int maxX = 0;

		for (int i = 0; i < points.size(); i++) {

			if (points.get(i)[0] < minX) {
				minX = (int) points.get(i)[0];
			}

			if (points.get(i)[0] > maxX) {
				maxX = (int) points.get(i)[0];
			}

		}

		return (maxX - minX) / 2;

	}

	public int getSecondaryRadius() {

		// allignAxis();

		int minY = Integer.MAX_VALUE;
		int maxY = 0;

		for (int i = 0; i < points.size(); i++) {

			if (points.get(i)[1] < minY) {
				minY = (int) points.get(i)[1];
			}

			if (points.get(i)[1] > maxY) {
				maxY = (int) points.get(i)[1];
			}
		}
		return (maxY - minY) / 2;
	}

	public Cluster[] divideCluster() {

		List<double[]> left = new ArrayList<double[]>();
		List<double[]> right = new ArrayList<double[]>();

		Cluster rotatedCluster = new Cluster(this);
		rotatedCluster.allignAxis();

		for (int i = 0; i < this.getPoints().size(); i++) {
			if (rotatedCluster.getPoints().get(i)[0] <= this.getCentroid()[0]) {

				left.add(this.getPoints().get(i));
			} else {

				right.add(this.getPoints().get(i));
			}
		}

		return new Cluster[] { new Cluster(left), new Cluster(right) };
	}

	public Cluster mergeClusters(Cluster c1) {

		List<double[]> points = new ArrayList<double[]>();

		points.addAll(this.getPoints());
		points.addAll(c1.getPoints());

		return new Cluster(points);
	}

	public List<double[]> getPoints() {
		return points;
	}

	public double[] getCentroid() {
		return centroid;
	}

	public double getOrientation() {
		return orientation;
	}

}
