package LargestRigidPart;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import NonRigid2D.Cluster1;
import ij.IJ;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class ClosestPoint {

	private Cluster c_i;
	private Cluster c_j;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;
	private double distanceThreshold = Input.distanceThresholdICP;
	private double distanceTotal = 0;

	private Map<Integer, Integer> sourceAssociation;
	private Map<Integer, Integer> targetAssociation;

	private List<double[]> referencePoints;
	private List<double[]> finalReferencePoints;
	private List<double[]> targetPoints;
	private List<double[]> finalTargetPoints;

	private Association associations;

	protected class Association {
		protected List<double[]> referencePoints;
		protected List<double[]> targetPoints;
		protected List<double[]> originalReference;
		protected List<double[]> originalTarget;
		private Map<Integer, Integer> reference;
		private Map<Integer, Integer> target;

		public Association(Map<Integer, Integer> reference, Map<Integer, Integer> target, List<double[]> originalReference, List<double[]> originalTarget) {
			this.reference = reference;
			this.target = target;
			this.originalReference = originalReference;
			this.originalTarget = originalTarget;
			comparePoints();
		}

		public void comparePoints() {

			referencePoints = new ArrayList<>();
			targetPoints = new ArrayList<>();

			for (Map.Entry<Integer, Integer> entry : reference.entrySet()) {
				Integer referenceIndex = entry.getKey();
				Integer targetIndex = entry.getValue();

				if (targetIndex != -1 && target.containsKey(targetIndex)) {
					if (target.get(targetIndex) == referenceIndex) {
						IJ.log("Association between point nr. " + referenceIndex + " and point nr. " + targetIndex);
						referencePoints.add(originalReference.get(referenceIndex));
						targetPoints.add(originalTarget.get(targetIndex));
					}
				}
			}
		}
	}

	public ClosestPoint(Cluster c_i, Cluster c_j) {
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		run();
	}

	private void run() {
		/*
		 * TODO: Orient around "joint" instead of centroid
		 */


		//initialOrientation();
		
		c_i.alignAxis();
		c_i.alignAxis(-c_j.getOrientation());

		referencePoints = Matrix.translate(c_i.getPoints(), c_j.getCentroid()[0] - c_i.getCentroid()[0],
				c_j.getCentroid()[1] - c_i.getCentroid()[1]);
		targetPoints = c_j.getPoints();

		// point association
		sourceAssociation = new HashMap<>();
		targetAssociation = new HashMap<>();
		ProcrustesFit pro = new ProcrustesFit();

		int iterations = 0;

		while (tmp_error <= error && iterations < 10) {

			ColorProcessor test = new ColorProcessor(Segmentation.width, Segmentation.height);
			test.invert();

			sourceAssociation = getAssociation(referencePoints, targetPoints);
			targetAssociation = getAssociation(targetPoints, referencePoints);

			if (sourceAssociation.size() == 0 || targetAssociation.size() == 0) {
				IJ.log("Size is 0 --> Return!");
				return;
			}

			associations = getAssociatedPoints(sourceAssociation, targetAssociation);

			IJ.log("Associated points calculated!");

			if (associations.referencePoints.size() == 0) {
				IJ.log("Size is 0 --> Return!");
				return;
			}
			Visualize.drawPoints(test, referencePoints, Color.black);
			Visualize.drawPoints(test, targetPoints, Color.blue);
			Visualize.drawPoints(test, associations.referencePoints, Color.red);
			Visualize.drawPoints(test, associations.targetPoints, Color.green);
			Visualize.showImage(test, "Iteration: " + iterations);

			// calculate transformation between reference and target points
			pro.fit(associations.targetPoints, associations.referencePoints);
			tmp_error = pro.getError();
			IJ.log("error: " + tmp_error);

			IJ.log("Rotation:" + pro.getR().getEntry(0, 0));
			IJ.log("Transformation: " + pro.getT().getEntry(0) + "/" + pro.getT().getEntry(1));

			if (tmp_error < error) {
				error = tmp_error;
				finalReferencePoints = associations.referencePoints;
				finalTargetPoints = associations.targetPoints;
			}

			double[] centroid = calculateCentroid(finalReferencePoints);

			// apply transformation from procrustes
			referencePoints = Matrix.translate(referencePoints, -centroid[0], -centroid[1]);
			referencePoints = Matrix.rotate(referencePoints, Math.acos(pro.getR().getEntry(0, 0)));
			referencePoints = Matrix.translate(referencePoints, centroid[0] + pro.getT().getEntry(0),
					centroid[1] + pro.getT().getEntry(1));

			iterations++;
		}

		finalReferencePoints = Matrix.translate(finalReferencePoints, c_i.getCentroid()[0] - c_j.getCentroid()[0],
				c_i.getCentroid()[1] - c_j.getCentroid()[1]);

		if (Input.showAssociations) {
			Visualize.drawAssociations(Segmentation.finalAssoc, finalReferencePoints, finalTargetPoints);
			Visualize.drawPoints(Segmentation.finalAssoc, finalReferencePoints, Color.blue);
			Visualize.drawPoints(Segmentation.finalAssoc, finalTargetPoints, Color.red);
		}
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param originalPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private Map<Integer, Integer> getAssociation(List<double[]> originalPositions, List<double[]> targetPositions) {
		Map<Integer, Integer> associations = new HashMap<>();
		distanceTotal = 0.0;

		for (int i = 0; i < originalPositions.size(); i++) {
			int closestPoint = closestPoint(originalPositions.get(i), targetPositions);
			associations.put(i, closestPoint);
		}
		
		return associations;
	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param referencePoints
	 * @return
	 */

	private int closestPoint(double[] point, List<double[]> referencePoints) {
		int closestPoint = -1;
		double distance = Double.MAX_VALUE;
		double distanceNew = 0;

		for (int i = 0; i < referencePoints.size(); i++) {

			distanceNew = Math.pow(point[0] - referencePoints.get(i)[0], 2)
					+ Math.pow(point[1] - referencePoints.get(i)[1], 2);

			if (distanceNew < distance && Math.sqrt(distanceNew) < distanceThreshold) {
				distance = distanceNew;
				closestPoint = i;
			}
		}
		distanceTotal += distanceNew;
		
		return closestPoint;
	}

	public double getError() {
		return error;
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

	private void initialOrientation() {
		c_i.alignAxis();
		Cluster rotation1 = new Cluster(c_i);
		Cluster rotation2 = new Cluster(c_i);

		rotation1.alignAxis();
		rotation1.alignAxis(-c_j.getOrientation());

		rotation2.alignAxis();
		rotation2.alignAxis(c_j.getOrientation());
		
		IJ.log("Rotation 1: " + rotation1.getOrientation());
		IJ.log("Rotation 2: " + rotation2.getOrientation());
		
		IJ.log("BEGIN: " + distanceTotal);
		
		getAssociation(rotation1.getPoints(), c_j.getPoints());
		//double error1 = distanceTotal;
		
		IJ.log("Error 1: " + distanceTotal);
		
		getAssociation(rotation2.getPoints(), c_j.getPoints());
		//double error2 = distanceTotal;
		
		IJ.log("Error 2: " + distanceTotal);

		if (true) {
			c_i = rotation1;
		}
		else {
			c_i = rotation2;
		}
	}

	private Association getAssociatedPoints(Map<Integer, Integer> reference, Map<Integer, Integer> target) {
		return new Association(reference, target, c_i.getPoints(), targetPoints);
	}

	public List<double[]> getTargetPoints() {
		return finalTargetPoints;
	}

	public List<double[]> getReferencePoints() {
		return finalReferencePoints;
	}

}
