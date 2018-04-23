package NonRigid2D;

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

	private Cluster c_i;
	private Cluster c_j;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;
	private double errorThreshold = Input.errorThreshold;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;

	private List<double[]> tmp_transformation;
	private List<double[]> tmp_association;

	private int amountC1;
	private int amountC2;
	private int amountPoints;

	public ClosestPoint(Cluster c_i, Cluster c_j) {
		this.c_i = c_i;
		this.c_j = c_j;

		amountC1 = c_i.getPoints().size();
		amountC2 = c_j.getPoints().size();

		IJ.log("Points in C1: " + amountC1);
		IJ.log("Points in C2: " + amountC2);

		amountPoints = Math.min(amountC1, amountC2);
		run();
	}

	private void run() {

		if (amountC1 < 2 || amountC2 < 2) {
			return;
		}

		/*
		 * Step 1 - initial transformation estimate (move centroids to origin)
		 */

		List<double[]> points1_origin = Matrix.translate(c_i.getPoints(), -c_i.getCentroid()[0], -c_i.getCentroid()[1]);
		List<double[]> points2_origin = Matrix.translate(c_j.getPoints(), -c_j.getCentroid()[0], -c_j.getCentroid()[1]);

		/*
		 * Step 2 - associate Points
		 */

		tmp_association = new ArrayList<double[]>();
		tmp_transformation = new ArrayList<double[]>();

		double i = 0;

		while (i < 360) {
			// recalculate Transformation for "best fit"
			tmp_transformation = Matrix.rotate(points1_origin, (i / 180.0) * Math.PI);
			tmp_association = getAssociation(tmp_transformation, points2_origin);

			if (tmp_error < error) {
				error = tmp_error;
				associatedPoints = tmp_association;
				transformedPoints = tmp_transformation;
			}
			i += 0.5;
		}

		associatedPoints = Matrix.translate(associatedPoints, c_j.getCentroid()[0], c_j.getCentroid()[1]);		
		transformedPoints = Matrix.translate(transformedPoints, c_j.getCentroid()[0], c_j.getCentroid()[1]);
		
		
		if(match()){
			if(Input.showAssociations){
				Visualize.drawAssociations(Segmentation.finalAssoc, transformedPoints, associatedPoints);
				Visualize.drawPoints(Segmentation.finalAssoc, transformedPoints, Color.black);
				Visualize.drawPoints(Segmentation.finalAssoc, c_j.getPoints(), Color.red);
			}		
		}

		ProcrustesFit pro = new ProcrustesFit();
		pro.fit(c_i.getPoints(), associatedPoints);
		// error = pro.getError();

		// IJ.log("Rotation:" + pro.getR().getEntry(0, 0));
		// IJ.log("Transformation: " + pro.getT().getEntry(0) + "/" +
		// pro.getT().getEntry(1));

		// transformedPoints = Matrix.rotate(points1_origin,
		// Math.acos(pro.getR().getEntry(0, 0)));
		// transformedPoints = Matrix.translate(transformedPoints, c1[0] +
		// pro.getT().getEntry(0), c1[1] + pro.getT().getEntry(1));
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param resultPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private List<double[]> getAssociation(List<double[]> resultPositions, List<double[]> targetPositions) {
		tmp_error = 0;
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

	private double[] closestPoint(double[] point, List<double[]> comparePoints) {

		double[] closestPoint = null;
		double distance = Double.MAX_VALUE;

		for (int i = 0; i < comparePoints.size(); i++) {

			double distanceNew =
					Math.pow(point[0] - comparePoints.get(i)[0], 2) + Math.pow(point[1] - comparePoints.get(i)[1], 2);

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = comparePoints.get(i);
			}
		}

		tmp_error += distance;

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
			return true;
		}

		double errorPerPoint = error / amountPoints;
		IJ.log("error per point: " + errorPerPoint);

		if (errorPerPoint < errorThreshold) {
			return true;
		}

		return false;

	}

}
