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
public class PartDetection {

	private Cluster c_i;
	private Cluster c_j;
	
	private Cluster[] linkedRigidParts;
	Cluster biggestClusterRef = new Cluster();
	Cluster biggestClusterTarget = new Cluster();

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0.0;
	private double distanceThreshold = Input.distanceThresholdICP;

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
		private double totalError = 0.0;

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

		public double getError() {
			return totalError;
		}

		public void comparePoints() {
			referencePoints = new ArrayList<>();
			targetPoints = new ArrayList<>();
			finalAssociations = new HashMap<>();

			for (Map.Entry<Integer, Integer> entry : reference.entrySet()) {
				Integer referenceIndex = entry.getKey();
				Integer targetIndex = entry.getValue();

				ClusterPoint referencePoint = originalReference.get(referenceIndex);
				ClusterPoint targetPoint = originalTarget.get(targetIndex);

				if (c_i.getJoint() != null) {
					double currentError = referencePoint.distance(targetPoint);
					totalError += currentError * (1/Math.pow(c_i.getJoint().distance(referencePoint),2));
				}

				if (target.containsKey(targetIndex)) {
					if ((reciprocalMatching && target.get(targetIndex) == referenceIndex) || !reciprocalMatching) {
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

	public PartDetection(Cluster c_i, Cluster c_j) {
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		run();
	}

	private void run() {

		IJ.log("Joint rotation entered!");

		if (c_i.getJoint() != null) {
//			initialOrientation(c_i.getJoint());

			referencePoints = Matrix.translate(c_i.getPoints(), -c_i.getJoint().getX(), -c_i.getJoint().getY());
			targetPoints = Matrix.translate(c_j.getPoints(), -c_j.getJoint().getX(), -c_j.getJoint().getY());
		}

		else {
			return;
		}

		// point association
		sourceAssociation = new HashMap<>();
		targetAssociation = new HashMap<>();
		finalTransformedPoints = new ArrayList<>();
		ColorProcessor results;

		int iterations = 0;
		int bestIteration = 1;

		while (iterations++ <= 360) {
			referencePoints = Matrix.rotate(referencePoints, (1 / 180.0) * Math.PI);

			sourceAssociation = getAssociation(referencePoints, targetPoints);
			targetAssociation = getAssociation(targetPoints, referencePoints);
			associations = getAssociatedPoints(sourceAssociation, targetAssociation);

			double tmp_error = associations.getError();

			results = new ColorProcessor(Main.width, Main.height);
			results.invert();

			if (tmp_error < error) {
				bestIteration = iterations;
				error = tmp_error;
				finalReferenceAssoc = associations.referencePoints;
				finalTargetAssoc = associations.targetPoints;
				finalTransformedPoints = referencePoints;
				finalAssociation = associations.getAssociations();
			}
		}

		IJ.log("best iteration: " + bestIteration);
		
		findBiggestCluster(finalAssociation);

		results = new ColorProcessor(Main.width, Main.height);
		results.invert();

		finalReferenceAssoc = Matrix.translate(finalReferenceAssoc, 400, 200);
		finalTargetAssoc = Matrix.translate(finalTargetAssoc, 400.0, 200);
		finalTransformedPoints = Matrix.translate(finalTransformedPoints, 400, 200);
		targetPoints = Matrix.translate(targetPoints, 400, 200);

		if (Input.showAssociations) {
			Visualize.drawAssociations(results, finalReferenceAssoc, finalTargetAssoc);
		}

		Visualize.drawPoints(results, finalTransformedPoints, Color.blue);
		Visualize.drawPoints(results, targetPoints, Color.red);

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
	private Map<Integer, Integer> getAssociation(List<ClusterPoint> originalPositions,
			List<ClusterPoint> targetPositions) {
		Map<Integer, Integer> associations = new HashMap<>();

		for (int i = 0; i < originalPositions.size(); i++) {
			int closestPoint = closestPoint(originalPositions.get(i), targetPositions);

			if (closestPoint != -1) {
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

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = i;
			}
		}

		tmp_error += distanceNew;
		return closestPoint;
	}
	
	private final void findBiggestCluster(Map<Integer, Integer> associations) {
		List<ClusterPoint> ref = new ArrayList<>();
		List<ClusterPoint> target = new ArrayList<>();

		for (Map.Entry<Integer, Integer> entry : associations.entrySet()) {
			ref.add(c_i.getPoints().get(entry.getKey()));
			target.add(c_j.getPoints().get(entry.getValue()));
		}
		for (Cluster cluster : RegionGrowing.detectClusters(ref)) {
			if (cluster.getPoints().size() > biggestClusterRef.getPoints().size()) {
				biggestClusterRef = cluster;
			}
		}
		for (Cluster cluster : RegionGrowing.detectClusters(target)) {
			if (cluster.getPoints().size() > biggestClusterTarget.getPoints().size()) {
				biggestClusterTarget = cluster;
			}
		}
		linkedRigidParts = new Cluster[] { biggestClusterRef, biggestClusterTarget };
	}

	public double getError() {
		return error;
	}

//	private void initialOrientation(ClusterPoint rotationPoint) {
//		Cluster rotation1 = new Cluster(c_i);
//		Cluster rotation2 = new Cluster(c_i);
//
//		rotation1.alignAxis(rotationPoint);
//		rotation1.alignAxis(c_j.getOrientation(), rotationPoint);
//
//		rotation2.alignAxis(rotationPoint);
//		rotation2.alignAxis(c_j.getOrientation() + Math.PI, rotationPoint);
//
//		getAssociation(rotation1.getPoints(), c_j.getPoints());
//		double error1 = tmp_error;
//
//		getAssociation(rotation2.getPoints(), c_j.getPoints());
//		double error2 = tmp_error;
//
//		if (error1 < error2) {
//			c_i = rotation1;
//		} else {
//			c_i = rotation2;
//		}
//	}

	private Association getAssociatedPoints(Map<Integer, Integer> reference, Map<Integer, Integer> target) {
		return new Association(reference, target, referencePoints, targetPoints);
	}
	
	public Cluster[] getLinkedParts(){
		return linkedRigidParts;
	}

	public Map<Integer, Integer> getCorrespondences() {
		return finalAssociation;
	}
}
