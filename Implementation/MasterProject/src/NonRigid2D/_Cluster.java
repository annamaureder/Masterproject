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
 * @author AM
 *
 */

public class _Cluster {

	private List<double[]> points = new ArrayList<double[]>();
	private double distanceThreshold = 10;

	private double[] centroid;
	private double orientation;
	//private int axisLength = 100;
	private int principalLength;
	private int secondaryLength;
	private boolean removeNoise;

	public _Cluster(ImageProcessor ip, boolean removeNoise) {
		this.removeNoise = removeNoise;
		this.points = getPoints(ip);
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	public _Cluster(List<double[]> p) {
		this.points = p;
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	/*
	 * copy constructor for temporal orientations
	 */
	public _Cluster(_Cluster c) {
		this.points = c.points;
		this.centroid = c.centroid;
		this.orientation = c.orientation;
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
			IJ.log("Removing done");
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

	public List<double[]> alignAxis() {

		points = _Matrix.translate(points, -centroid[0], -centroid[1]);
		points = _Matrix.rotate(points, -orientation);
		points = _Matrix.translate(points, centroid[0], centroid[1]);

		orientation = 0;

		return points;
	}

	public void drawPrincipalAxis(ImageProcessor cp) {
		
		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];

		int x_test = (int) centroid[0] - 30;

		int x2 = (int) (x1 + principalLength * Math.cos(orientation));
		int y2 = (int) (y1 + principalLength * Math.sin(orientation));
		int x2_ = (int) (x1 - principalLength * Math.cos(orientation));
		int y2_ = (int) (y1 - principalLength * Math.sin(orientation));

		int y = (int) (Math.sin(orientation) * (x_test - x1) + y1);

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
	}

	public void drawSecondaryAxis(ImageProcessor cp) {
		
		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];

		int x2 = (int) (x1 + secondaryLength * Math.cos(orientation + 1.5707963268));
		int y2 = (int) (y1 + secondaryLength * Math.sin(orientation + 1.5707963268));
		int x2_ = (int) (x1 - secondaryLength * Math.cos(orientation + 1.5707963268));
		int y2_ = (int) (y1 - secondaryLength * Math.sin(orientation + 1.5707963268));

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);

	}

	public int getPrincipalRadius() {
		
		_Cluster copy = new _Cluster(this);
		copy.alignAxis();
		// alignAxis();

		int minX = Integer.MAX_VALUE;
		int maxX = 0;

		for (int i = 0; i < copy.points.size(); i++) {

			if (copy.points.get(i)[0] < minX) {
				minX = (int) copy.points.get(i)[0];
			}

			if (copy.points.get(i)[0] > maxX) {
				maxX = (int) copy.points.get(i)[0];
			}

		}

		return (maxX - minX) / 2;

	}

	public int getSecondaryRadius() {

		_Cluster copy = new _Cluster(this);
		copy.alignAxis();

		int minY = Integer.MAX_VALUE;
		int maxY = 0;

		for (int i = 0; i < copy.points.size(); i++) {

			if (copy.points.get(i)[1] < minY) {
				minY = (int) copy.points.get(i)[1];
			}

			if (copy.points.get(i)[1] > maxY) {
				maxY = (int) copy.points.get(i)[1];
			}
		}
		return (maxY - minY) / 2;
	}

	public _Cluster[] divideCluster() {

		List<double[]> left = new ArrayList<double[]>();
		List<double[]> right = new ArrayList<double[]>();

		_Cluster rotatedCluster = new _Cluster(this);
		rotatedCluster.alignAxis();

		for (int i = 0; i < this.getPoints().size(); i++) {
			if (rotatedCluster.getPoints().get(i)[0] <= this.getCentroid()[0]) {

				left.add(this.getPoints().get(i));
			} else {

				right.add(this.getPoints().get(i));
			}
		}

		return new _Cluster[] { new _Cluster(left), new _Cluster(right) };
	}

	public _Cluster mergeClusters(_Cluster c1) {

		List<double[]> points = new ArrayList<double[]>();

		points.addAll(this.getPoints());
		points.addAll(c1.getPoints());

		return new _Cluster(points);
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
