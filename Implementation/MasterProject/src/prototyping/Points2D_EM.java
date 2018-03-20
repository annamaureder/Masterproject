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

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class Points2D_EM implements PlugInFilter {

	// variable declaration

	private List<double[]> points1;
	private List<double[]> points2;

	private List<double[]> part1 = new ArrayList<double[]>();
	private List<double[]> part2 = new ArrayList<double[]>();
	private List<double[]> associated1 = new ArrayList<double[]>();
	private List<double[]> associated2 = new ArrayList<double[]>();

	private int divideOffset = 0;

	private PointsOrientation o1;
	private PointsOrientation o2;
	private boolean running = true;

	private double error = Double.MAX_VALUE;

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
		o1.drawSecondaryAxis(cp);
		o2.drawSecondaryAxis(cp);
		showImage(cp, "Point cloud orientation");

		/*
		 * Orient the point clouds around their centroids, that both principal
		 * axis overlap
		 */

		points1 = o1.allignAxis();
		points2 = o2.allignAxis();

		/*
		 * ToDo:
		 * 
		 * - segment both point clouds in two point clusters (axis perpendicular
		 * to principal axis) - apply ICP on both clusters from points1 that
		 * they match on the two clusters of points2 - "shift" dividing axis in
		 * the direction of most errors to get different clusters (take care of
		 * amount of points) --> until ICP error gets worse (threshold)
		 * 
		 */

		int direction = 0;

		int i = 0;

		while (i < 20 && running) {

			List<List<double[]>> seperated = seperateList(points1, direction);
			List<double[]> points1_1 = seperated.get(0);
			List<double[]> points1_2 = seperated.get(1);

			List<List<double[]>> seperated2 = seperateList(points2, direction);
			List<double[]> points2_1 = seperated2.get(0);
			List<double[]> points2_2 = seperated2.get(1);

			PointsOrientation o1_1 = new PointsOrientation(points1_1);
			PointsOrientation o2_1 = new PointsOrientation(points2_1);
			PointsOrientation o1_2 = new PointsOrientation(points1_2);
			PointsOrientation o2_2 = new PointsOrientation(points2_2);

			ColorProcessor sep = new ColorProcessor(ip.getWidth(), ip.getHeight());
			sep.invert();

			ICP icp1 = new ICP(points1_1, points2_1);
			double error1 = icp1.getError();

			ICP icp2 = new ICP(points1_2, points2_2);
			double error2 = icp2.getError();

			double tmp_error = error1 + error2;

			if (true) {

				error = tmp_error;

				part1 = points1_1;
				part2 = points1_2;
				associated1 = icp1.getAssociatedPoints();
				associated2 = icp2.getAssociatedPoints();

				IJ.log("Error1: " + error1);
				IJ.log("Error2: " + error2);
				IJ.log("error: " + error);

				// draw the new segmentation

				/*drawCentroid(sep, o1_1.getCentroid(), Color.red, 5);
				drawCentroid(sep, o2_1.getCentroid(), Color.red, 5);
				drawCentroid(sep, o1_2.getCentroid(), Color.red, 5);
				drawCentroid(sep, o2_2.getCentroid(), Color.red, 5);
				drawPoints(sep, points1_1, Color.red);
				drawPoints(sep, points2_1, Color.red);
				drawPoints(sep, points1_2, Color.blue);
				drawPoints(sep, points2_2, Color.blue);
				o1_1.drawPrincipalAxis(sep);
				o2_1.drawPrincipalAxis(sep);
				o1_2.drawPrincipalAxis(sep);
				o2_2.drawPrincipalAxis(sep);*/
				
				ColorProcessor assoc = new ColorProcessor(ip.getWidth(), ip.getHeight());
				assoc.invert();

				drawAssociations(assoc, part1, associated1);
				drawAssociations(assoc, part2, associated2);
				drawPoints(assoc, points1, Color.black);
				drawPoints(assoc, points2, Color.black);
				// drawPoints(assoc, icp1.getTransformedPoints(), Color.red);
				// drawPoints(assoc, icp2.getTransformedPoints(), Color.red);

				ColorProcessor segmentation = new ColorProcessor(ip.getWidth(), ip.getHeight());
				segmentation.invert();

				drawPoints(segmentation, part1, Color.red);
				drawPoints(segmentation, part2, Color.blue);
				drawPoints(segmentation, associated1, Color.red);
				drawPoints(segmentation, associated2, Color.blue);

				showImage(assoc, "Point association");
				showImage(segmentation, "Segmentation");

				showImage(sep, "Point cloud orientation");

				if (i == 0)
					direction = (int) (error2 - error1);

			}

			else {

				running = false;

			}

			i++;

			IJ.log("iteation nr. " + i);

		}

		/*
		 * 
		 * 
		 * Draw associated points from part1 and part2
		 * 
		 * 
		 */

		ColorProcessor assoc = new ColorProcessor(ip.getWidth(), ip.getHeight());
		assoc.invert();

		drawAssociations(assoc, part1, associated1);
		drawAssociations(assoc, part2, associated2);
		drawPoints(assoc, points1, Color.black);
		drawPoints(assoc, points2, Color.black);
		// drawPoints(assoc, icp1.getTransformedPoints(), Color.red);
		// drawPoints(assoc, icp2.getTransformedPoints(), Color.red);

		ColorProcessor segmentation = new ColorProcessor(ip.getWidth(), ip.getHeight());
		segmentation.invert();

		drawPoints(segmentation, part1, Color.red);
		drawPoints(segmentation, part2, Color.blue);
		drawPoints(segmentation, associated1, Color.red);
		drawPoints(segmentation, associated2, Color.blue);

		showImage(assoc, "Point association");
		showImage(segmentation, "Segmentation");
	}

	private List<List<double[]>> seperateList(List<double[]> points1, int direction) {

		List<double[]> leftPart = new ArrayList<double[]>();
		List<double[]> rightPart = new ArrayList<double[]>();

		if (direction < 0) {
			divideOffset++;
		}

		else if (direction > 0) {
			divideOffset--;
		}

		double[] centroid = PointCollection.calculateCentroid(points1);

		for (double[] point : points1) {
			if (point[0] + divideOffset <= centroid[0]) {
				leftPart.add(point);

			}

			else {
				rightPart.add(point);
			}
		}

		List seperated = new ArrayList<List<double[]>>();
		seperated.add(leftPart);
		seperated.add(rightPart);

		return seperated;
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
