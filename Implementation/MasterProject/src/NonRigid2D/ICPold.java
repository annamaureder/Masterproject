package NonRigid2D;

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


/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class ICPold implements PlugInFilter {
	
	//variable declaration
	
	private List<double[]> points1;
	private List<double[]> points2;
	private boolean running = true;

	private double[] initialTransformation = new double[2];
	private double error = Double.MAX_VALUE;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;
	
	private PointsOrientation o;
	private PointsOrientation o2;
	

	ImagePlus im;
	
	//setup

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
		
		o = new PointsOrientation(points1);
		o2 = new PointsOrientation(points2);
		
		//draw points clouds with their principal axis
		
		ColorProcessor cp = ip.convertToColorProcessor();
		
		drawCentroid(cp, o.getCentroid(), Color.red, 5);
		drawCentroid(cp, o2.getCentroid(), Color.red, 5);
		drawPoints(cp, points1, Color.black);
		drawPoints(cp, points2, Color.black);
		o.drawAxis(cp);
		o2.drawAxis(cp);
		
		/*
		 * Orient the point clouds around their centroids, that both principal axis overlap
		 */
		
		//points1 = o.rotateAroundCentroid(o.getOrientation()*-1);
		//points2 = o2.rotateAroundCentroid(o2.getOrientation()*-1);
		
		List<double[]> transformed = new ArrayList<double[]>();
	
		
		/*
		 * Step 1 - initial transformation estimate
		 */
		
		initialTransformation = calculateInitialTransformation(points1, points2);
		IJ.log("initial transformation: " + initialTransformation[0] + "/" + initialTransformation[1]);

		/*
		 * Step 2 - associate Points, Recalculate T by the Procrustes method
		 */

		double[][] r = new double[][] { { 1.0, 0, 0 }, { 0, 1.0, 0 }, { 0, 0, 1.0 } };
		List<double[]> association = null;
		
		int x = 0;
		
		while (running) {

			IJ.log("new round!");
			associatedPoints = new ArrayList<double[]>();
			transformedPoints = new ArrayList<double[]>();

			double[][] transformMatrix = new double[][] { { r[0][0], r[0][1], initialTransformation[0] },
					{ r[1][0], r[1][1], initialTransformation[1] }, { 0, 0, 1.0 } };
					
			double[] transformedPoint;

			for (int i = 0; i < points1.size(); i++) {

				transformedPoint = Matrix.multiplication(transformMatrix, points1.get(i));
				transformedPoints.add(transformedPoint);
			}

			associatedPoints = getAssociation(transformedPoints, points2);

			//recalculate Transformation for "best fit"
			
			ProcrustesFit pro = new ProcrustesFit();
			pro.fit(points1, associatedPoints);

			double tmp_error = pro.getError();
			RealMatrix rotation = pro.getR();
			RealVector transformation = pro.getT();

			initialTransformation[0] = transformation.getEntry(0);
			initialTransformation[1] = transformation.getEntry(1);

			r[0] = rotation.getRow(0);
			r[1] = rotation.getRow(1);
			//r[2] = rotation.getRow(2);
			
			if (tmp_error < error) {
				error = tmp_error;
				association = associatedPoints;
				transformed = transformedPoints;
				
				IJ.log("new transformation: " + initialTransformation[0] + "/" + initialTransformation[1]);
				IJ.log("Rotation r1: " + r[0][0] + " " + r[0][1]);
				IJ.log("Rotation r2: " + r[1][0] + " " + r[1][1]);
				
			}

			else if (tmp_error >= error) {
				IJ.log("Best fit found! " + x);
				running = false;
			}

			IJ.log("Error: " + error);
			
			x++;
			
		}

		/*
		 * 
		 * Step 3: Draw points, orientations and asssociations
		 * 
		 */
		
		ColorProcessor assoc = new ColorProcessor(ip.getWidth(), ip.getHeight());
		assoc.invert();
		
		drawAssociations(assoc, points1, association);
		drawPoints(assoc, points1, Color.black);
		drawPoints(assoc, points2, Color.black);
		drawPoints(assoc, transformed, Color.red);

		showImage(cp, "Point cloud orientation");
		showImage(assoc, "Point association");
	}


	// -------------------------------------------------------------------

	private void drawCentroid(ColorProcessor ip, double[] point, Color color, int size){
		
		int x = (int)point[0];
		int y = (int)point[1];
		
		ip.setColor(color);
		ip.drawLine(x - size/2, y, x + size/2, y);
		ip.drawLine(x, y - size/2, x, y + size/2);
		
	}
	
	
	private void drawPoints(ColorProcessor ip, List<double[]> points, Color color) {
		ip.setColor(color);
		for (double[] point : points) {
			ip.drawDot((int)point[0], (int)point[1]);
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

			Path2D line = new Path2D.Double();
			line.moveTo(points1.get(i)[0], points1.get(i)[1]);
			line.lineTo(association.get(i)[0], association.get(i)[1]);
			ShapeRoi roi1 = new ShapeRoi(line);
			roi1.setStrokeWidth(0.2f);
			roi1.setStrokeColor(Color.green);
			ip.draw(roi1);
		}

	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param resultPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	public List<double[]> getAssociation(List<double[]> resultPositions, List<double[]> targetPositions) {

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

		return closestPoint;
	}

	/**
	 * method for the least square fit to find the best matching association
	 * between set X and X'
	 * 
	 * @param assocPoints
	 * @param resultPoints
	 * @return
	 */

	private double[] calculateInitialTransformation(List<double[]> p1, List<double[]> p2) {

		double[] centroid1 = PointCollection.calculateCentroid(p1);
		double[] centroid2 = PointCollection.calculateCentroid(p2);
		
		initialTransformation[0] = centroid2[0] - centroid1[0];
		initialTransformation[1] = centroid2[1] - centroid1[1];
		
		return initialTransformation;

	}
	
	void showImage(ImageProcessor ip, String title) {
		(new ImagePlus(title, ip)).show();
	}
}
