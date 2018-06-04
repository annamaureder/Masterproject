package LargestRigidPart;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import ij.IJ;
import ij.process.ColorProcessor;

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class PartDetection {

	private Cluster c_i;
	private Cluster c_j;

	private double x_location;
	private double y_location;

	private Cluster[] linkedRigidParts;
	Cluster biggestClusterRef = new Cluster();
	Cluster biggestClusterTarget = new Cluster();

	private double error = Double.MAX_VALUE;
	private double distanceThreshold = Input.distanceThresholdJoints;

	ColorProcessor results;
	ColorProcessor finalCluster;
	List<PointCorrespondence> pointCorrespondences = new ArrayList<>();
	List<PointCorrespondence> finalPointCorrespondences = new ArrayList<>();

	private List<ClusterPoint> referencePoints;
	private List<ClusterPoint> targetPoints;
	private List<ClusterPoint> finalTransformedPoints;

	private double maxDistanceToJoint;

	/**
	 * nested class to represent a point correspondence between to clusters in
	 * the form of indices
	 * 
	 * @author Anna
	 */
	protected class PointCorrespondence {
		private int referenceIndex;
		private int targetIndex;
		double distance;

		public PointCorrespondence(int reference, int target, double distance) {
			this.referenceIndex = reference;
			this.targetIndex = target;
			this.distance = distance;
		}

		@Override
		public boolean equals(Object o) {
			boolean result;
			if ((o == null) || (getClass() != o.getClass())) {
				result = false;
			} else {
				PointCorrespondence other = (PointCorrespondence) o;
				result = this.referenceIndex == other.referenceIndex && this.targetIndex == other.targetIndex;
			}
			return result;
		}

		public String toString() {
			return "Reference Index: " + referenceIndex + "\n" + "Target index: " + targetIndex + "\n" + "Distance: "
					+ distance;
		}
	}

	public PartDetection(Cluster c_i, Cluster c_j) {
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);
		maxDistanceToJoint = distanceToJoint();
		run();
	}

	public double comparePoints(List<PointCorrespondence> pointCorrespondences) {
		double totalError = 0.0;

		for (PointCorrespondence pointCorrespondence : pointCorrespondences) {
			ClusterPoint referencePoint = referencePoints.get(pointCorrespondence.referenceIndex);

			double distanceToJoint = referencePoint.distance(new ClusterPoint(0, 0));
			double currentError = pointCorrespondence.distance;
			totalError += currentError * Math.pow((1 - distanceToJoint / maxDistanceToJoint), 2);

		}
		return totalError;
	}

	private void run() {

		IJ.log("Joint rotation entered!");

		if (c_i.getJoint() == null) {
			return;
		}

		x_location = c_i.getJoint().getX();
		y_location = c_i.getJoint().getY();
		initialOrientation(c_i.getJoint());
		referencePoints = Matrix.translate(c_i.getPoints(), -c_i.getJoint().getX(), -c_i.getJoint().getY());
		targetPoints = Matrix.translate(c_j.getPoints(), -c_j.getJoint().getX(), -c_j.getJoint().getY());

		finalTransformedPoints = new ArrayList<>();

		int iterations = 0;
		double tmp_error = 0.0;

		int direction = getRotationDirection();
		IJ.log("Direction: " + direction);
		int i = 0;

		while (tmp_error <= error) {
			IJ.log("Iteration: " + i++);

			referencePoints = Matrix.rotate(referencePoints, (direction / 180.0) * Math.PI);
			pointCorrespondences = getCorrespondences(referencePoints, targetPoints);

			tmp_error = comparePoints(pointCorrespondences);

			IJ.log("Error Nr." + iterations + ": " + tmp_error);

			drawIntermediateResult(iterations);

			if (tmp_error < error) {
				error = tmp_error;
				finalTransformedPoints = referencePoints;
			}
		}

		findBiggestCluster(finalTransformedPoints);

		results = new ColorProcessor(Main.width, Main.height);
		results.invert();

		finalCluster = new ColorProcessor(Main.width, Main.height);
		finalCluster.invert();

		finalTransformedPoints = Matrix.translate(finalTransformedPoints, x_location, y_location);
		targetPoints = Matrix.translate(targetPoints, x_location, y_location);

		// if (Input.showAssociations) {
		// Visualize.drawAssociations(results, finalReferenceAssoc,
		// finalTargetAssoc);
		// }

		Visualize.drawPoints(results, finalTransformedPoints, Color.red);
		Visualize.drawPoints(results, targetPoints, Color.blue);

		Visualize.drawPoints(finalCluster, linkedRigidParts[0].getPoints(), Color.red);
		Visualize.drawPoints(finalCluster, linkedRigidParts[1].getPoints(), Color.blue);

		String fileName = "LRP_" + distanceThreshold + "th_" + "iterations" + iterations;
		Visualize.addToResults(results, fileName);
		Visualize.addToResults(finalCluster, "Detected linked part");
	}

	private void drawIntermediateResult(int iterations) {
		List<ClusterPoint> reference = new ArrayList<>();
		List<ClusterPoint> target = new ArrayList<>();

		ColorProcessor test = new ColorProcessor(Main.width, Main.height);
		test.invert();

		for (PointCorrespondence pc : pointCorrespondences) {
			reference.add(referencePoints.get(pc.referenceIndex));
			target.add(targetPoints.get(pc.targetIndex));
		}

		Visualize.drawDot(test, c_i.getJoint(), Color.green, 10);
		Visualize.drawPoints(test, Matrix.translate(referencePoints, x_location, y_location), Color.red);
		Visualize.drawPoints(test, Matrix.translate(targetPoints, x_location, y_location), Color.blue);
		Visualize.drawAssociations(test, Matrix.translate(reference, x_location, y_location),
				Matrix.translate(target, x_location, y_location));
		Visualize.addToResults(test, "#" + iterations);
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param originalPositions
	 *            X
	 * @param targetPositions
	 *            X'
	 * @return List with point correspondences in form of indices
	 */
	private List<PointCorrespondence> getCorrespondences(List<ClusterPoint> originalPositions,
			List<ClusterPoint> targetPositions) {

		List<PointCorrespondence> correspondences = new ArrayList<>();

		for (int i = 0; i < originalPositions.size(); i++) {
			double distance = Double.MAX_VALUE;
			double distanceNew = 0;
			int closestPoint = 0;

			int j;
			for (j = 0; j < targetPositions.size(); j++) {
				distanceNew = originalPositions.get(i).distance(targetPositions.get(j));

				if (distanceNew < distance) {
					distance = distanceNew;
					closestPoint = j;
				}
			}
			correspondences.add(new PointCorrespondence(i, closestPoint, distance));
		}

		IJ.log("Size correspondences: " + correspondences.size());

		return correspondences;
	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param referencePoints
	 * @return
	 */

	private final void findBiggestCluster(List<ClusterPoint> finalReferencePoints) {
		List<ClusterPoint> ref = new ArrayList<>();
		List<ClusterPoint> target = new ArrayList<>();

		List<PointCorrespondence> corRef = getCorrespondences(finalReferencePoints, targetPoints);
		List<PointCorrespondence> corTarget = getCorrespondences(targetPoints, finalReferencePoints);

		IJ.log("Cluster size: " + c_j.getPoints().size());
		IJ.log("correspondences size: " + corTarget.size());

		for (PointCorrespondence correspondence : corRef) {
			if (correspondence.distance < distanceThreshold) {
				ref.add(c_i.getPoints().get(correspondence.referenceIndex));
			}
		}

		for (PointCorrespondence correspondence : corTarget) {
			if (correspondence.distance < distanceThreshold) {
				target.add(c_j.getPoints().get(correspondence.referenceIndex));
			}
		}

		ColorProcessor cp = new ColorProcessor(Main.width, Main.height);
		cp.invert();
		Visualize.drawPoints(cp, ref, Color.red);
		Visualize.drawPoints(cp, target, Color.blue);
		Visualize.addToResults(cp, "Input for region growing");

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

		biggestClusterRef.setJoint(c_i.getJoint());
		biggestClusterTarget.setJoint(c_j.getJoint());

		linkedRigidParts = new Cluster[] { biggestClusterRef, biggestClusterTarget };
	}

	public double getError() {
		return error;
	}

	private void initialOrientation(ClusterPoint rotationPoint) {
		// Cluster rotation1 = new Cluster(c_i);
		// Cluster rotation2 = new Cluster(c_i);
		//
		// rotation1.alignAxis(rotationPoint);
		// rotation1.alignAxis(c_j.getOrientation(), rotationPoint);
		//
		// rotation2.alignAxis(rotationPoint);
		// rotation2.alignAxis(c_j.getOrientation() + Math.PI, rotationPoint);
		//

		// getAssociation(rotation1.getPoints(), c_j.getPoints());
		// double error1 = tmp_error;
		//
		// getAssociation(rotation2.getPoints(), c_j.getPoints());
		// double error2 = tmp_error;

		// if (error1 < error2) {
		// c_i = rotation1;
		// } else {
		// c_i = rotation2;
		// }
	}

	private int getRotationDirection() {
		List<ClusterPoint> direction1;
		List<ClusterPoint> direction2;

		double error1;
		double error2;

		direction1 = Matrix.rotate(referencePoints, (1 / 180.0) * Math.PI);
		pointCorrespondences = getCorrespondences(direction1, targetPoints);
		error1 = comparePoints(pointCorrespondences);

		direction2 = Matrix.rotate(referencePoints, (-1 / 180.0) * Math.PI);
		pointCorrespondences = getCorrespondences(direction2, targetPoints);
		error2 = comparePoints(pointCorrespondences);

		return error1 < error2 ? 1 : -1;
	}

	public Cluster[] getLinkedParts() {
		return linkedRigidParts;
	}

	private double distanceToJoint() {
		double maxDistance = Double.MIN_VALUE;

		for (ClusterPoint point : c_i.getPoints()) {
			double distance = point.distance(c_i.getJoint());
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}
		return maxDistance;
	}
}
