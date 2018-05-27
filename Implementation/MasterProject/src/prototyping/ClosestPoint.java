package prototyping;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import LargestRigidPart.Cluster;
import LargestRigidPart.ClusterPoint;
import LargestRigidPart.Input;
import LargestRigidPart.Matrix;
import LargestRigidPart.Main;
import LargestRigidPart.Visualize;
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
	private double distanceThreshold = Input.distanceThresholdJoints;
	private double distanceTotal = 0;

	private boolean logging = Input.logging;

	private Map<Integer, Integer> sourceAssociation;
	private Map<Integer, Integer> targetAssociation;
	private Map<Integer, Integer> finalAssociation;

	private List<ClusterPoint> referencePoints;
	private List<ClusterPoint> targetPoints;

	private List<ClusterPoint> finalReferenceAssoc;
	private List<ClusterPoint> finalTargetAssoc;

	private List<ClusterPoint> finalTransformedPoints;

	private Association associations;
	private boolean reciprocalMatching = Input.reciprocalMatching;

	protected class Association {
		protected List<ClusterPoint> referencePoints;
		protected List<ClusterPoint> targetPoints;
		protected List<ClusterPoint> originalReference;
		protected List<ClusterPoint> originalTarget;
		private Map<Integer, Integer> reference;
		private Map<Integer, Integer> target;
		private Map<Integer, Integer> finalAssociations;

		public Association(Map<Integer, Integer> reference, Map<Integer, Integer> target,
				List<ClusterPoint> originalReference, List<ClusterPoint> originalTarget) {
			this.reference = reference;
			this.target = target;
			this.originalReference = originalReference;
			this.originalTarget = originalTarget;
			comparePoints();
		}

		public Map<Integer, Integer> getAssociations() {
			return finalAssociations;
		}

		public void comparePoints() {
			referencePoints = new ArrayList<>();
			targetPoints = new ArrayList<>();
			finalAssociations = new HashMap<>();

			for (Map.Entry<Integer, Integer> entry : reference.entrySet()) {
				Integer referenceIndex = entry.getKey();
				Integer targetIndex = entry.getValue();

				if (target.containsKey(targetIndex)) {
					if (reciprocalMatching && target.get(targetIndex) == referenceIndex) {
						if (logging)
							IJ.log("Association between point nr. " + referenceIndex + " and point nr. " + targetIndex);
						referencePoints.add(originalReference.get(referenceIndex));
						targetPoints.add(originalTarget.get(targetIndex));
						finalAssociations.put(referenceIndex, targetIndex);
					} else {
						if (logging)
							IJ.log("Association between point nr. " + referenceIndex + " and point nr. " + targetIndex);
						referencePoints.add(originalReference.get(referenceIndex));
						targetPoints.add(originalTarget.get(targetIndex));
						finalAssociations.put(referenceIndex, targetIndex);
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

		// align joints of c_1 and c_2
		if (c_i.getJoint() != null) {			
			initialOrientation(c_i.getJoint());

			referencePoints = Matrix.translate(c_i.getPoints(), c_j.getJoint().getX() - c_i.getJoint().getX(),
					c_j.getJoint().getY() - c_i.getJoint().getY());
		} else {
			//TODO: initial alignment!!!
			// c_i.alignAxis(c_i.getCentroid());
			// c_i.alignAxis(c_j.getOrientation(), c_i.getCentroid());

			// referencePoints = Matrix.translate(c_i.getPoints(),
			// c_j.getCentroid()[0] - c_i.getCentroid()[0],
			// c_j.getCentroid()[1] - c_i.getCentroid()[1]);

			referencePoints = Matrix.translate(c_i.getPoints(), 0, 0);
		}

		targetPoints = c_j.getPoints();

		// point association
		sourceAssociation = new HashMap<>();
		targetAssociation = new HashMap<>();
		finalTransformedPoints = new ArrayList<>();
		ProcrustesFit pro = new ProcrustesFit();
		ColorProcessor results;

		int iterations = 0;

		while (tmp_error <= error && iterations < 10) {

			results = new ColorProcessor(Main.width, Main.height);
			results.invert();

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
			Visualize.drawPoints(results, referencePoints, Color.black);
			Visualize.drawPoints(results, targetPoints, Color.blue);
			Visualize.drawPoints(results, associations.referencePoints, Color.red);
			Visualize.drawPoints(results, associations.targetPoints, Color.green);
			Visualize.showImage(results, "Iteration: " + iterations);

			// calculate transformation between reference and target points
			pro.fit(convertClusterPoints(associations.targetPoints), convertClusterPoints(associations.referencePoints));
			tmp_error = pro.getError();

			IJ.log("error: " + tmp_error);
			IJ.log("Rotation:" + pro.getR().getEntry(0, 0));
			IJ.log("Transformation: " + pro.getT().getEntry(0) + "/" + pro.getT().getEntry(1));

			ClusterPoint centroid = calculateCentroid(associations.referencePoints);
			
			if(c_i.getJoint() != null){
				centroid = c_i.getJoint();
			}

			// apply transformation from procrustes
			referencePoints = Matrix.translate(referencePoints, -centroid.getX(), -centroid.getY());
			referencePoints = Matrix.rotate(referencePoints, Math.acos(pro.getR().getEntry(0, 0)));
//			referencePoints = Matrix.translate(referencePoints, centroid[0] + pro.getT().getEntry(0),
//					centroid[1] + pro.getT().getEntry(1));
			referencePoints = Matrix.translate(referencePoints, centroid.getX(),
					centroid.getY());

			if (tmp_error < error) {
				error = tmp_error;
				finalReferenceAssoc = associations.referencePoints;
				finalTargetAssoc = associations.targetPoints;
				finalTransformedPoints = referencePoints;
				finalAssociation = associations.getAssociations();
			}
			iterations++;
		}

		results = new ColorProcessor(Main.width, Main.height);
		results.invert();

		Visualize.drawPoints(results, finalTransformedPoints, Color.blue);
		Visualize.drawPoints(results, targetPoints, Color.red);

		if (Input.showAssociations) {
			Visualize.drawAssociations(results, finalReferenceAssoc, finalTargetAssoc);
		}

		String fileName = "LRP_" + distanceThreshold + "th_" + "iterations" + iterations;
		if (reciprocalMatching) {
			fileName += "_reciprocal";
		}
		Visualize.showImage(results, fileName);
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param originalPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private Map<Integer, Integer> getAssociation(List<ClusterPoint> originalPositions, List<ClusterPoint> targetPositions) {
		Map<Integer, Integer> associations = new HashMap<>();
		distanceTotal = 0.0;

		for (int i = 0; i < originalPositions.size(); i++) {
			int closestPoint = closestPoint(originalPositions.get(i), targetPositions);

			double distanceToJoint = 0;
			double distanceJointCentroid = 0;

			if (c_i.getJoint() != null) {
				distanceToJoint = originalPositions.get(i).distance(c_i.getJoint());
				distanceJointCentroid = c_i.getCentroid().distance(c_i.getJoint());
			}
			if (closestPoint != -1 && distanceToJoint <= distanceJointCentroid) {
				associations.put(i, closestPoint);
			}
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

	private int closestPoint(ClusterPoint point, List<ClusterPoint> referencePoints) {
		int closestPoint = -1;
		double distance = Double.MAX_VALUE;
		double distanceNew = 0;

		for (int i = 0; i < referencePoints.size(); i++) {
			distanceNew = point.distance(referencePoints.get(i));

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

	private ClusterPoint calculateCentroid(List<ClusterPoint> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i).getX();
			avgY += points.get(i).getY();
		}
		return new ClusterPoint(avgX / points.size(), avgY / points.size());
	}

	private void initialOrientation(ClusterPoint rotationPoint) {
		Cluster rotation1 = new Cluster(c_i);
		Cluster rotation2 = new Cluster(c_i);

		rotation1.alignAxis(rotationPoint);
		rotation1.alignAxis(c_j.getOrientation(), rotationPoint);

		rotation2.alignAxis(rotationPoint);
		rotation2.alignAxis(c_j.getOrientation() + Math.PI, rotationPoint);

		IJ.log("Rotation 1: " + rotation1.getOrientation());
		IJ.log("Rotation 2: " + rotation2.getOrientation());

		getAssociation(rotation1.getPoints(), c_j.getPoints());
		double error1 = distanceTotal;

		IJ.log("Error 1: " + distanceTotal);

		getAssociation(rotation2.getPoints(), c_j.getPoints());
		double error2 = distanceTotal;

		IJ.log("Error 2: " + distanceTotal);

		if (error1 < error2) {
			c_i = rotation1;
		} else {
			c_i = rotation2;
		}
	}

	private Association getAssociatedPoints(Map<Integer, Integer> reference, Map<Integer, Integer> target) {
		return new Association(reference, target, referencePoints, targetPoints);
	}

	public Map<Integer, Integer> getCorrespondences() {
		return finalAssociation;
	}
	
	private List<double[]> convertClusterPoints(List<ClusterPoint> clusterPoints){
		List<double[]> finalPoints = new ArrayList<>();
		for(ClusterPoint point : clusterPoints){
			finalPoints.add(new double[]{point.getX(), point.getY()});
		}
		return finalPoints;
	}
}
