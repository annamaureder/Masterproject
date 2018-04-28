package prototyping;

import ij.IJ;
import ij.ImagePlus;
import ij.ImageStack;
import ij.gui.GenericDialog;
import ij.gui.Overlay;
import ij.gui.ShapeRoi;
import ij.plugin.filter.PlugInFilter;
import ij.process.ColorProcessor;
import ij.process.ImageProcessor;
import imagingbook.pub.corners.Corner;
import imagingbook.pub.corners.HarrisCornerDetector;
import procrustes.ProcrustesFit;

import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.apache.commons.math3.geometry.spherical.twod.Vertex;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import NonRigid2D.Matrix1;

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class Points2D_LRP implements PlugInFilter {

	// variable declaration

	private List<double[]> points1;
	private List<double[]> points2;

	private double numberAssoc = 0;
	private double tmp_numberAssoc = 0;
	private final double pointThreshold = 5;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;

	private PointsOrientation o1;
	private PointsOrientation o2;

	ImagePlus im;

	// setup

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		return DOES_ALL + NO_CHANGES;
	}

	public void run(ImageProcessor ip) {

		ImageStack stack = im.getStack();
		ImageProcessor ip1 = stack.getProcessor(1);
		ImageProcessor ip2 = stack.getProcessor(2);

		points1 = PointCollection.getPoints(ip1);
		points2 = PointCollection.getPoints(ip2);

		o1 = new PointsOrientation(points1);
		o2 = new PointsOrientation(points2);

		// draw points clouds with their principal axis

		ColorProcessor cp = ip.convertToColorProcessor();

		drawCentroid(cp, o1.getCentroid(), Color.red, 5);
		drawCentroid(cp, o2.getCentroid(), Color.red, 5);
		drawPoints(cp, points1, Color.black);
		drawPoints(cp, points2, Color.black);
		o1.drawPrincipalAxis(cp);
		o2.drawPrincipalAxis(cp);

		/*
		 * Orient the point clouds around their centroids, that both principal
		 * axis overlap
		 */

		points1 = o1.allignAxis();
		points2 = o2.allignAxis();

		// calculate the bounding box point xMin, yMax

		double[] minBB1 = o1.getMinBB();
		double[] minBB2 = o2.getMinBB();

		// minBB1[1] = o1.getCentroid()[1];
		// minBB2[1] = o2.getCentroid()[1];

		List<double[]> transformed = new ArrayList<double[]>();

		/*
		 * Step 1 - initial transformation estimate (move centroids to origin)
		 */

		// shift both point clouds to the origin (centroid, bounding box point
		// as origin)

		List<double[]> points1_origin = Matrix1.translate(points1, -o1.getCentroid()[0], -o1.getCentroid()[1]);
		List<double[]> points2_origin = Matrix1.translate(points2, -o2.getCentroid()[0], -o2.getCentroid()[1]);

		// List<double[]> points1_origin = Matrix.translate(points1, -minBB1[0],
		// -minBB1[1]);
		// List<double[]> points2_origin = Matrix.translate(points2, -minBB2[0],
		// -minBB2[1]);

		/*
		 * Step 2 - associate Points
		 */

		List<double[]> association = null;
		transformedPoints = new ArrayList<double[]>();

		int i = 0;

		while (i < 360) {

			IJ.log("new round!");

			// recalculate Transformation for "best fit"

			transformedPoints = Matrix1.rotate(points1_origin, (i / 180.0) * Math.PI);
			associatedPoints = getAssociation(transformedPoints, points2_origin);

			if (tmp_numberAssoc > numberAssoc) {
				numberAssoc = tmp_numberAssoc;
				association = associatedPoints;
				transformed = transformedPoints;
				IJ.log("number of points associated: " + numberAssoc);
				IJ.log("Rotation: " + i);
			}
			i++;
		}
		
	
		/*
		 * 
		 * Step 3: Draw points, orientations and asssociations
		 * 
		 */

		// shift the points back from the origin

		association = Matrix1.translate(association, o2.getCentroid()[0], o2.getCentroid()[1]);
		transformed = Matrix1.translate(transformed, o2.getCentroid()[0], o2.getCentroid()[1]);
		
		
		//filter all points that have a corresponding point
		
		List<double[]> points1_filtered = new ArrayList<double[]>();
		List<double[]> points_filtered = new ArrayList<double[]>();

		for (int x = 0; x < association.size(); x++) {
			
			double[] point = association.get(x);

			if (point != null){
				IJ.log("assoc: " + point[0] + "/" + point[1]);
				points_filtered.add(point);
				points1_filtered.add(points1.get(x));
				
			}

			else{
				IJ.log("NULL");
			}

		}
		
		//use points for RANSAC to get transformation with most point matches --> LRP
		
		/*
		 * ToDo: 
		 * - select clusters (neighboring points with associations)
		 * - Take three random points from both clusters (points1 and points2)
		 * - apply procrustes fit to get the best transformation
		 * - apply transformation on all points and count amount of matching points
		 * 
		 */
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		//Draw all associated points
		
		ColorProcessor assocPoints = new ColorProcessor(ip.getWidth(), ip.getHeight());
		assocPoints.invert();
		
		drawPoints(assocPoints, points1_filtered, Color.black);
		drawPoints(assocPoints, points_filtered, Color.black);
		
		showImage(assocPoints, "filteredPoints");
				
		// association = Matrix.translate(association, minBB2[0], minBB2[1]);
		// transformed = Matrix.translate(transformed, minBB2[0], minBB2[1]);

		ColorProcessor assoc = new ColorProcessor(ip.getWidth(), ip.getHeight());
		assoc.invert();

		drawAssociations(assoc, points1, association);
		drawPoints(assoc, points1, Color.black);
		drawPoints(assoc, points2, Color.black);
		drawPoints(assoc, transformed, Color.red);
		drawCentroid(assoc, minBB1, Color.green, 3);
		drawCentroid(assoc, minBB2, Color.green, 3);

		showImage(cp, "Point cloud orientation");
		showImage(assoc, "Point association");
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param resultPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	public List<double[]> getAssociation(List<double[]> resultPositions, List<double[]> targetPositions) {

		tmp_numberAssoc = 0;
		List<double[]> assocPoints = new ArrayList<double[]>();

		for (int i = 0; i < resultPositions.size(); i++) {
			assocPoints.add(closestPoint(resultPositions.get(i), targetPositions));
		}

		return assocPoints;

	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param comparePoints
	 * @return
	 */

	public double[] closestPoint(double[] point, List<double[]> comparePoints) {

		double[] closestPoint = null;
		double distance = Double.MAX_VALUE;

		for (int i = 0; i < comparePoints.size(); i++) {

			double distanceNew = Math.sqrt(
					Math.pow(point[0] - comparePoints.get(i)[0], 2) + Math.pow(point[1] - comparePoints.get(i)[1], 2));

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = comparePoints.get(i);
			}
		}

		if (distance <= pointThreshold) {
			tmp_numberAssoc++;
		}

		else {
			closestPoint = null;
		}

		return closestPoint;
	}

	// -------------------------------------------------------------------

	private void drawCentroid(ColorProcessor ip, double[] point, Color color, int size) {

		int x = (int) point[0];
		int y = (int) point[1];

		ip.setColor(color);
		ip.drawLine(x - size / 2, y, x + size / 2, y);
		ip.drawLine(x, y - size / 2, x, y + size / 2);

	}

	private void drawPoints(ColorProcessor ip, List<double[]> points, Color color) {
		ip.setColor(color);
		for (double[] point : points) {
			ip.drawDot((int) point[0], (int) point[1]);
		}
	}

	/**
	 * method to draw lines between the associated points of the sets X and X'
	 * 
	 * @param ip
	 * @param cornersLeft
	 * @param association
	 */
	private void drawAssociations(ColorProcessor ip, List<double[]> points1, List<double[]> association) {

		ip.setColor(Color.green);

		for (int i = 0; i < points1.size(); i++) {

			if (association.get(i) != null) {

				Path2D line = new Path2D.Double();
				line.moveTo(points1.get(i)[0], points1.get(i)[1]);
				line.lineTo(association.get(i)[0], association.get(i)[1]);
				ShapeRoi roi1 = new ShapeRoi(line);
				roi1.setStrokeWidth(0.2f);
				roi1.setStrokeColor(Color.green);
				ip.draw(roi1);

			}
		}

	}

	/**
	 * method for the least square fit to find the best matching association
	 * between set X and X'
	 * 
	 * @param assocPoints
	 * @param resultPoints
	 * @return
	 */

	void showImage(ImageProcessor ip, String title) {
		(new ImagePlus(title, ip)).show();
	}
}
