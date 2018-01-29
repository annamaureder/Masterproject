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

	private Cluster C1;
	private Cluster C2;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;
	private double thresholdError = 5;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;

	private List<double[]> tmp_transformation;
	private List<double[]> tmp_association;

	private int amountC1;
	private int amountC2;
	private int amountPoints;

	public ClosestPoint(Cluster C1, Cluster C2) {

		this.C1 = C1;
		this.C2 = C2;

		amountC1 = C2.getPoints().size();
		amountC2 = C2.getPoints().size();

		IJ.log("amount1: " + amountC1);
		IJ.log("amount2: " + amountC2);

		amountPoints = Math.min(amountC1, amountC2);

		this.run();
	}

	private void run() {

		if (amountC1 < 2 || amountC2 < 2) {
			return;
		}

		/*
		 * Step 1 - initial transformation estimate (move centroids to origin)
		 */

		List<double[]> points1_origin = Matrix.translate(C1.getPoints(), -C1.getCentroid()[0], -C1.getCentroid()[1]);
		List<double[]> points2_origin = Matrix.translate(C2.getPoints(), -C2.getCentroid()[0], -C2.getCentroid()[1]);

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

		IJ.log("error: " + error);

		associatedPoints = Matrix.translate(associatedPoints, C2.getCentroid()[0], C2.getCentroid()[1]);
		transformedPoints = Matrix.translate(transformedPoints, C2.getCentroid()[0], C2.getCentroid()[1]);
		
		
		if(match()){
			
			Visualize.drawAssociations(Segmentation.finalAssoc, transformedPoints, associatedPoints);
			Visualize.drawPoints(Segmentation.finalAssoc, transformedPoints, Color.black);
			Visualize.drawPoints(Segmentation.finalAssoc, C2.getPoints(), Color.red);
			
			//not matching parts
			
		/*ColorProcessor cp = new ColorProcessor(Segmentation.size, Segmentation.size);
		cp.invert();
		
		Visualize.drawAssociations(cp, transformedPoints, associatedPoints);
		Visualize.drawPoints(cp, transformedPoints, Color.black);
		Visualize.drawPoints(cp, C2.getPoints(), Color.red);
		
		Visualize.showImage(cp, "Associations");*/
		
		}

		ProcrustesFit pro = new ProcrustesFit();
		pro.fit(C1.getPoints(), associatedPoints);
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

			double distanceNew = Math.sqrt(
					Math.pow(point[0] - comparePoints.get(i)[0], 2) + Math.pow(point[1] - comparePoints.get(i)[1], 2));

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

		if (amountC1 < 2 || amountC2 < 2) {
			return true;
		}

		double errorPerPoint = error / amountPoints;
		IJ.log("error per point: " + errorPerPoint);

		if (errorPerPoint < thresholdError) {
			return true;
		}

		return false;

	}

}
