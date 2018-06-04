package LargestRigidPart;

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

public class Cluster {

	private List<ClusterPoint> points = new ArrayList<>();
	private int size;

	private ClusterPoint centroid;
	private double resolution;
	private ClusterPoint joint;
	private double orientation;
	private int principalLength;
	private int secondaryLength;
	private final int normalNeighbors = 2;
	private final int featureNeighbors = 5;

	public Cluster(ImageProcessor ip) {
		this.points = getPoints(ip);
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
		this.resolution = calculateResolution();
	}

	public Cluster(List<ClusterPoint> p) {
		this.points = p;
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
		this.resolution = calculateResolution();
	}

	public Cluster() {

	}

	/*
	 * copy constructor for temporal orientations
	 */
	public Cluster(Cluster c) {
		this.points.addAll(c.points);
		this.joint = c.joint;
		this.size = c.size;
		this.centroid = c.centroid;
		this.orientation = c.orientation;
		this.resolution = c.resolution;
	}

	private List<ClusterPoint> getPoints(ImageProcessor ip) {
		int w = ip.getWidth();
		int h = ip.getHeight();

		List<ClusterPoint> pntlist = new ArrayList<>();

		for (int v = 0; v < h; v++) {
			for (int u = 0; u < w; u++) {
				int p = ip.getPixel(u, v);
				if (p < -1) {
					pntlist.add(new ClusterPoint(u, v));
				}
			}
		}

		if (Input.removeOutliers) {
			IJ.log("Remove Noise");
			List<Cluster> clusters = RegionGrowing.detectClusters(pntlist);
			IJ.log("Removing done");
			return clusters.get(0).points;
		}
		return pntlist;
	}

	private ClusterPoint calculateCentroid(List<ClusterPoint> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i).getX();
			avgY += points.get(i).getY();
		}
		return new ClusterPoint(avgX / points.size(), avgY / points.size());
	}

	private double calculateOrientation() {
		return Math.atan2((2 * centralMoment(1, 1)), centralMoment(2, 0) - centralMoment(0, 2)) / 2.0;
	}

	private double centralMoment(int p, int q) {
		double moment = 0.0;

		for (ClusterPoint point : points) {
			moment += Math.pow((point.getX() - centroid.getX()), p) * Math.pow((point.getY() - centroid.getY()), q);
		}
		return moment;
	}

	public List<ClusterPoint> alignAxis(double orientation, ClusterPoint rotationPoint) {
		points = Matrix.translate(points, -rotationPoint.getX(), -rotationPoint.getY());
		points = Matrix.rotate(points, orientation);
		points = Matrix.translate(points, rotationPoint.getX(), rotationPoint.getY());
		this.orientation += orientation;

		return points;
	}

	public List<ClusterPoint> alignAxis(ClusterPoint rotationPoint) {
		return alignAxis(-this.orientation, rotationPoint);
	}

	public void drawPrincipalAxis(ImageProcessor cp) {
		int x1 = (int) centroid.getX();
		int y1 = (int) centroid.getY();

		int x_test = (int) centroid.getX() - 30;

		int x2 = (int) (x1 + principalLength * Math.cos(orientation));
		int y2 = (int) (y1 + principalLength * Math.sin(orientation));
		int x2_ = (int) (x1 - principalLength * Math.cos(orientation));
		int y2_ = (int) (y1 - principalLength * Math.sin(orientation));

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
	}

	public void drawSecondaryAxis(ImageProcessor cp) {
		int x1 = (int) centroid.getX();
		int y1 = (int) centroid.getY();

		int x2 = (int) (x1 + secondaryLength * Math.cos(orientation + 1.5707963268));
		int y2 = (int) (y1 + secondaryLength * Math.sin(orientation + 1.5707963268));
		int x2_ = (int) (x1 - secondaryLength * Math.cos(orientation + 1.5707963268));
		int y2_ = (int) (y1 - secondaryLength * Math.sin(orientation + 1.5707963268));

		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
	}

	public int getPrincipalRadius() {
		Cluster copy = new Cluster(this);
		copy.alignAxis(copy.centroid);

		int minX = Integer.MAX_VALUE;
		int maxX = 0;

		for (int i = 0; i < copy.points.size(); i++) {

			if (copy.points.get(i).getX() < minX) {
				minX = (int) copy.points.get(i).getX();
			}

			if (copy.points.get(i).getY() > maxX) {
				maxX = (int) copy.points.get(i).getY();
			}

		}
		return (maxX - minX) / 2;
	}

	public int getSecondaryRadius() {
		Cluster copy = new Cluster(this);
		copy.alignAxis(copy.centroid);

		int minY = Integer.MAX_VALUE;
		int maxY = 0;

		for (int i = 0; i < copy.points.size(); i++) {

			if (copy.points.get(i).getY() < minY) {
				minY = (int) copy.points.get(i).getY();
			}

			if (copy.points.get(i).getY() > maxY) {
				maxY = (int) copy.points.get(i).getY();
			}
		}
		return (maxY - minY) / 2;
	}

	private int getMinPoint(Cluster c) {
		ClusterPoint minPoint = c.getPoints().get(0);
		int minPointIndex = 0;
		int i = 0;

		for (ClusterPoint point : c.getPoints()) {
			if (point.getX() < minPoint.getX()) {
				minPoint = point;
				minPointIndex = i;
			}
			i++;
		}
		return minPointIndex;
	}

	private List<ClusterPoint> getNeighborhood(ClusterPoint point, int number) {
		return RegionGrowing.nearestNeighbors(points, point, number).get(0).getPoints();
	}

	private double calculateResolution() {
		double resolution = 0;
		int number = 0;
		int maxPoints = 10;

		while (number < maxPoints) {
			int randomIndex = (int) (Math.random() * points.size());
			ClusterPoint current = points.get(randomIndex);

			double distance = Double.MAX_VALUE;

			for (ClusterPoint point : points) {
				double distanceNew = point.distance(current);

				if (distanceNew != 0.0 && distanceNew < distance) {
					distance = distanceNew;
				}
			}
			resolution += distance;
			number++;
		}
		return resolution / maxPoints;
	}

	public void calculateFeatures() {

		// PCA normals
		for (ClusterPoint point : points) {
			calculateNormal(point, getNeighborhood(point, normalNeighbors));
			point.setNeighborhood(getNeighborhood(point, featureNeighbors));
		}

		// orientNormals();

		for (int i = 0; i < points.size(); i++) {
			IJ.log("Features for point " + i + 1);
			FPFH feature = new FPFH(points.get(i));
			IJ.log("FPFH created!");
			feature.featureHistogram();
		}

		// points.get(0).getFPFH().showHistogram();
	}

	public void calculateNormal(ClusterPoint point, List<ClusterPoint> neighbors) {
		NormalEstimation n = new NormalEstimation(point, neighbors);
		n.estimateNormal();
	}

	public void orientNormals() {
		List<ClusterPoint> ref = new ArrayList<>();
		ref.addAll(points);

		ClusterPoint parent = getHighestPoint(ref);
		List<ClusterPoint> neighbors = new ArrayList<>();

		while (!ref.isEmpty()) {
			for (int i = 0; i < points.size(); i++) {
				ClusterPoint current = points.get(i);
				if (ref.contains(current) && parent.distance(current) < 15) {
					if (parent.dot(current) < 0) {
						points.get(i).setNormal(flip(current.getNormal()));
					}
					neighbors.add(current);
				}
			}
			parent = neighbors.get(0);
			ref.removeAll(neighbors);
			neighbors.remove(parent);
		}
	}

	public ClusterPoint getHighestPoint(List<ClusterPoint> points) {
		ClusterPoint maximum = points.get(0);

		for (ClusterPoint point : points) {
			if (point.getY() < maximum.getY()) {
				maximum = point;
			}
		}
		return maximum;
	}

	public double[] flip(double[] normal) {
		return new double[] { normal[0] * -1, normal[1] * -1 };
	}

	public List<ClusterPoint> getPoints() {
		return points;
	}

	public ClusterPoint getCentroid() {
		return centroid;
	}

	public double getOrientation() {
		return orientation;
	}

	public ClusterPoint getJoint() {
		return joint;
	}

	public void setJoint(ClusterPoint joint) {
		this.joint = joint;
	}
	
	public double getResoultion(){
		return this.resolution;
	}

	public List<Histogram> getHistograms() {
		List<Histogram> histograms = new ArrayList<>();

		for (ClusterPoint point : points) {
			histograms.add(point.getFPFH());
		}
		return histograms;
	}
}
