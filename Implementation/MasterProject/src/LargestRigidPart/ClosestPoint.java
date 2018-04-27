package LargestRigidPart;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;

import ij.IJ;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class ClosestPoint {

	// variable declaration

	private Cluster reference;
	
	private Cluster c_i;
	private Cluster c_j;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;
	private double errorThreshold = Input.errorThreshold;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;

	private List<double[]> transformation;
	private List<double[]> association;

	private int amountC1;
	private int amountC2;
	private int amountPoints;

	public ClosestPoint(Cluster c_i, Cluster c_j) {
		
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);
		this.reference = new Cluster(c_i);

		amountC1 = c_i.getPoints().size();
		amountC2 = c_j.getPoints().size();
		amountPoints = amountC1;
		run();
	}

	private void run() {
		if (amountC1 < 2 || amountC2 < 2) {
			return;
		}
		
		//orient c1 like c2
		c_i.alignAxis();
		c_i.alignAxis(-c_j.getOrientation());
		/*
		 * Step 1 - initial transformation estimate (move centroid of points1 to centroid of
		 * points2)
		 */

		List<double[]> points1 = Matrix.translate(c_i.getPoints(), c_j.getCentroid()[0] - c_i.getCentroid()[0],
				c_j.getCentroid()[1] - c_i.getCentroid()[1]);
		List<double[]> points2 = c_j.getPoints();
		
		/*
		 * Step 2 - associate Points
		 */
		transformation = points1;
		association = new ArrayList<double[]>();
		ProcrustesFit pro = new ProcrustesFit();
		
		int iterations = 0;
		
		while (!match() && iterations < 10) {
			association = getAssociation(transformation, points2);

			// calculate transformation between points1 and points2
			pro.fit(transformation, association);
			tmp_error = pro.getError();

			IJ.log("Rotation:" + pro.getR().getEntry(0, 0));
			IJ.log("Transformation: " + pro.getT().getEntry(0) + "/" + pro.getT().getEntry(1));

			if (tmp_error < error) {
				error = tmp_error;
				associatedPoints = association;
				transformedPoints = transformation;
			}
			double[] centroid = calculateCentroid(transformedPoints);
			
			// apply transformation from procrustes
			transformation = Matrix.translate(transformation, -centroid[0], -centroid[1]);
			transformation = Matrix.rotate(points1, Math.acos(pro.getR().getEntry(0, 0)));
			transformation = Matrix.translate(transformation, -centroid[0] + pro.getT().getEntry(0), -centroid[1]+pro.getT().getEntry(1));
		
			iterations++;
		}
		
		transformedPoints = Matrix.translate(transformedPoints, c_i.getCentroid()[0] - c_j.getCentroid()[0],
				c_i.getCentroid()[1] - c_j.getCentroid()[1]);

		if (match()) {
			if (Input.showAssociations) {
				Visualize.drawAssociations(Segmentation.finalAssoc, reference.getPoints(), associatedPoints);
				Visualize.drawPoints(Segmentation.finalAssoc, reference.getPoints(), Color.black);
				Visualize.drawPoints(Segmentation.finalAssoc, associatedPoints, Color.red);
			}
		}
	}
	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param originalPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private List<double[]> getAssociation(List<double[]> originalPositions, List<double[]> targetPositions) {
		List<double[]> assocPoints = new ArrayList<double[]>();

		for (int i = 0; i < originalPositions.size(); i++) {
			assocPoints.add(closestPoint(originalPositions.get(i), targetPositions));
		}
		return assocPoints;
	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param referencePoints
	 * @return
	 */

	private double[] closestPoint(double[] point, List<double[]> referencePoints) {
		double[] closestPoint = null;
		double distance = Double.MAX_VALUE;

		for (int i = 0; i < referencePoints.size(); i++) {

			double distanceNew = Math.pow(point[0] - referencePoints.get(i)[0], 2)
					+ Math.pow(point[1] - referencePoints.get(i)[1], 2);

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = referencePoints.get(i);
			}
		}
		return closestPoint;
	}

	public List<double[]> getAssociatedPoints() {
		return associatedPoints;
	}

	public List<double[]> getTransformedPoints() {
		return transformedPoints;
	}

	public double getError() {
		return error;
	}

	public boolean match() {
		if (amountC1 < 5 || amountC2 < 5) {
			IJ.log("Amount is too low!");
			return true;
		}

		double errorPerPoint = error / amountPoints;
		IJ.log("error per point: " + errorPerPoint);

		if (errorPerPoint < errorThreshold) {
			return true;
		}

		return false;
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

}
