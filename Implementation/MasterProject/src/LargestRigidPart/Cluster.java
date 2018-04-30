package LargestRigidPart;

import java.util.List;
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

public class Cluster implements Comparable<Cluster> {

	private List<double[]> points = new ArrayList<double[]>();
	private int size;

	private double[] centroid;
	private double[] joint;
	private double orientation;
	private int principalLength;
	private int secondaryLength;

	public Cluster(ImageProcessor ip) {
		this.points = getPoints(ip);
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}

	public Cluster(List<double[]> p) {
		this.points = p;
		this.size = points.size();
		this.centroid = calculateCentroid(points);
		this.orientation = calculateOrientation();
		this.principalLength = getPrincipalRadius();
		this.secondaryLength = getSecondaryRadius();
	}
	
	public Cluster(){
		
	}

	/*
	 * copy constructor for temporal orientations
	 */
	public Cluster(Cluster c) {
		this.points = c.points;
		this.joint = c.joint;
		this.size = c.size;
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
		
		if (Input.removeOutliers) {
			IJ.log("Remove Noise");
			List<Cluster> clusters = RegionGrowing.detectClusters(pntlist, null);
			Collections.sort(clusters);
			IJ.log("Removing done");
			return clusters.get(0).points;
		}
		return pntlist;
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

	public List<double[]> alignAxis(double orientation, double[] centroid) {
		points = Matrix.translate(points, -centroid[0], -centroid[1]);
		points = Matrix.rotate(points, orientation);
		points = Matrix.translate(points, centroid[0], centroid[1]);
		this.orientation += orientation;

		return points;
	}
	
	public List<double[]> alignAxis(double[] centroid){
		return alignAxis(-this.orientation, centroid);
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
		Cluster copy = new Cluster(this);
		copy.alignAxis(copy.centroid);

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
		Cluster copy = new Cluster(this);
		copy.alignAxis(copy.centroid);

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

	private int getMinPoint(Cluster c) {
		double[] minPoint = c.getPoints().get(0);
		int minPointIndex = 0;
		int i = 0;

		for (double[] point : c.getPoints()) {
			if (point[0] < minPoint[0]) {
				minPoint = point;
				minPointIndex = i; 
			}
			i++;
		}
		return minPointIndex;
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
	
	public double[] getJoint() {
		return joint;
	}
	
	public void setJoint(double[] joint){
		this.joint = joint;
	}

	@Override
	public int compareTo(Cluster c) {
		int size = c.points.size();
        return size - this.points.size();
	}

}
