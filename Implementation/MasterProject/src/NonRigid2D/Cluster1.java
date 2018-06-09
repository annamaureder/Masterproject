package NonRigid2D;

import java.util.List;

import LargestRigidPart.ClusterPoint;
import LargestRigidPart.Input;

import java.util.ArrayList;
import java.util.Collections;

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

public class Cluster1 {

	private List<ClusterPoint1> points = new ArrayList<>();
	private double distanceThreshold = Input1.distanceThreshold;
	private int size;

	private double resolution;
	private ClusterPoint1 centroid;
	private double orientation;
	private int principalLength;
	private int secondaryLength;
	private ClusterPoint1 joint;

	public Cluster1(ImageProcessor ip) {
		this.points = getPoints(ip);
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
		this.resolution = calculateResolution();
	}

	public Cluster1(List<ClusterPoint1> p) {
		this.points = p;
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
		this.resolution = calculateResolution();
	}

	/*
	 * copy constructor for temporal orientations
	 */
	public Cluster1(Cluster1 c) {
		this.points = c.points;
		this.size = c.size;
		this.centroid = c.centroid;
		this.orientation = c.orientation;
		this.resolution = c.resolution;
		this.joint = c.joint;
	}

	private List<ClusterPoint1> getPoints(ImageProcessor ip) {
		int w = ip.getWidth();
		int h = ip.getHeight();

		List<ClusterPoint1> pntlist = new ArrayList<>();

		for (int v = 0; v < h; v++) {
			for (int u = 0; u < w; u++) {
				int p = ip.getPixel(u, v);
				if (p < -1) {
					pntlist.add(new ClusterPoint1(u,v));
				}
			}
		}

		if (Input1.removeOutliers) {
			IJ.log("Remove Noise");
			pntlist = biggestCluster(pntlist);
			IJ.log("Removing done");
		}

		return pntlist;
	}

	private List<ClusterPoint1> biggestCluster(List<ClusterPoint1> allPoints) {
		List<ClusterPoint1> current = new ArrayList<>();
		List<ClusterPoint1> maximum = new ArrayList<>();

		while (!allPoints.isEmpty()) {
			current.add(allPoints.get(0));

			for (int c = 0; c < current.size(); c++) {
				allPoints.removeAll(current);
				for (ClusterPoint1 point: allPoints) {
					if (point.distance(current.get(c)) < distanceThreshold) {
						current.add(point);
					}
				}
			}
			allPoints.removeAll(current);

			if (current.size() > maximum.size()) {
				maximum = current;
			}
			current = new ArrayList<>();
		}
		return maximum;
	}

	private ClusterPoint1 calculateCentroid(List<ClusterPoint1> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (ClusterPoint1 point: points) {
			avgX += point.getX();
			avgY += point.getY();
		}

		return new ClusterPoint1(avgX / points.size(), avgY / points.size());
	}

	private double calculateOrientation() {
		return Math.atan2((2 * centralMoment(1, 1)), centralMoment(2, 0) - centralMoment(0, 2)) / 2.0;
	}

	private double centralMoment(int p, int q) {
		double moment = 0.0;

		for (ClusterPoint1 point : points) {
			moment += Math.pow((point.getX() - centroid.getX()), p) * Math.pow((point.getY() - centroid.getY()), q);
		}
		return moment;
	}

	public List<ClusterPoint1> alignAxis(double orientation) {
		points = Matrix1.translate(points, -centroid.getX(), -centroid.getY());
		points = Matrix1.rotate(points, orientation);
		points = Matrix1.translate(points, centroid.getX(), centroid.getY());
		orientation = 0;

		return points;
	}
	
	public List<ClusterPoint1> alignAxis(){
		return alignAxis(-this.orientation);
	}

	public void drawPrincipalAxis(ImageProcessor cp) {
		int x1 = (int) centroid.getX();
		int y1 = (int) centroid.getY();

		int x_test = (int) centroid.getX() - 30;

		int x2 = (int) (x1 + principalLength * Math.cos(orientation));
		int y2 = (int) (y1 + principalLength * Math.sin(orientation));
		int x2_ = (int) (x1 - principalLength * Math.cos(orientation));
		int y2_ = (int) (y1 - principalLength * Math.sin(orientation));

		int y = (int) (Math.sin(orientation) * (x_test - x1) + y1);

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
		Cluster1 copy = new Cluster1(this);
		copy.alignAxis();

		int minX = Integer.MAX_VALUE;
		int maxX = 0;

		for (ClusterPoint1 copyPoint : copy.points) {

			if (copyPoint.getX() < minX) {
				minX = (int) copyPoint.getX();
			}

			if (copyPoint.getX() > maxX) {
				maxX = (int) copyPoint.getX();
			}

		}
		return (maxX - minX) / 2;
	}

	public int getSecondaryRadius() {
		Cluster1 copy = new Cluster1(this);
		copy.alignAxis();

		int minY = Integer.MAX_VALUE;
		int maxY = 0;

		for (ClusterPoint1 copyPoint : copy.points) {

			if (copyPoint.getY() < minY) {
				minY = (int) copyPoint.getY();
			}

			if (copyPoint.getY() > maxY) {
				maxY = (int) copyPoint.getY();
			}
		}
		return (maxY - minY) / 2;
	}

	private int getMinPoint(Cluster1 c) {
		ClusterPoint1 minPoint = c.getPoints().get(0);
		int minPointIndex = 0;

		for (ClusterPoint1 point : c.getPoints()) {
			if (point.getX() < minPoint.getX()) {
				minPoint = point;
				minPointIndex = c.getPoints().indexOf(point);
			}
		}
		return minPointIndex;
	}
	
	private double calculateResolution() {
		int number = 0;
		int maxPoints = 10;
		
		List<Double> distances = new ArrayList<>();

		while (number++ < maxPoints) {
			int randomIndex = (int) (Math.random() * points.size());
			ClusterPoint1 current = points.get(randomIndex);

			double distance = Double.MAX_VALUE;
			double distanceNew = 0.0;

			for (ClusterPoint1 point : points) {
				distanceNew = point.distance(current);

				if (distanceNew != 0.0 && distanceNew < distance) {
					distance = distanceNew;
				}
			}
			distances.add(distance);
		}
		Collections.sort(distances);
		return distances.get(maxPoints/2 - 1);
	}

	public Cluster1[] divideCluster() {
		List<ClusterPoint1> left = new ArrayList<>();
		List<ClusterPoint1> right = new ArrayList<>();

		Cluster1 rotatedCluster = new Cluster1(this);
		rotatedCluster.alignAxis();

		if (Input1.regionGrowing) {
			int minPoint = getMinPoint(rotatedCluster);
			ClusterPoint1 leftStart = points.get(minPoint);
			right.addAll(this.points);
			left = regionGrowing(right, leftStart, this.points.size()/2);
			right.removeAll(left);
		}

		else {
			for (ClusterPoint1 point : points) {
				if (point.getX() <= centroid.getX()) {
					left.add(point);
				} else {
					right.add(point);
				}
			}
		}
		
		IJ.log("Points: " + this.points.size());
		IJ.log("left: " + left.size());
		IJ.log("right: " + right.size());
		
		return new Cluster1[] { new Cluster1(left), new Cluster1(right) };
	}
	
	private List<ClusterPoint1> regionGrowing(List<ClusterPoint1> points, ClusterPoint1 start, int maxAmount){
		List<ClusterPoint1> region = new ArrayList<>();
		region.add(start);
		
			for (ClusterPoint1 clusteredPoint : region) {
				points.removeAll(region);
				for (ClusterPoint1 point : points) {
					if (point.distance(clusteredPoint) < distanceThreshold) {
						region.add(point);
						if(region.size() == maxAmount){
							return region;
						}
					}
				}
			}
			return region;
		}

	public Cluster1 mergeClusters(Cluster1 c1) {
		List<ClusterPoint1> points = new ArrayList<>();

		points.addAll(this.getPoints());
		points.addAll(c1.getPoints());

		return new Cluster1(points);
	}

	public List<ClusterPoint1> getPoints() {
		return points;
	}

	public ClusterPoint1 getCentroid() {
		return centroid;
	}
	
	public double getResolution(){
		return resolution;
	}

	public double getOrientation() {
		return orientation;
	}
	
	public void setJoint(ClusterPoint1 joint){
		this.joint = joint;
	}
	
	public ClusterPoint1 getJoint(){
		return joint;
	}
	
	public void estimateJoint(Cluster1 c){
		double x = 0;
		double y = 0;
		int numberPoints = 0;
		
		for (ClusterPoint1 point1 : this.getPoints()) {
			for (ClusterPoint1 point2 : c.getPoints()) {
				if (point1.distance(point2) < Input1.distanceThreshold) {
					x += point1.getX() + point2.getX();
					y += point1.getY() + point2.getY();
					numberPoints += 2;
				}
			}
		}
		if (numberPoints == 0) {
			IJ.log("No joint could be calculated!");
			return;
		}
		this.joint = new ClusterPoint1(x / numberPoints, y / numberPoints);
	}

}
