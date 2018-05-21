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
public class ClosestPoint3 {

	private Cluster c_i;
	private Cluster c_j;

	private double error = Double.MAX_VALUE;
	private boolean logging = Input.logging;
	private Association associations;

	private Map<Integer, Integer> sourceAssociation;
	private Map<Integer, Integer> targetAssociation;

	private List<ClusterPoint> referencePoints;
	private List<ClusterPoint> targetPoints;

	private boolean reciprocalMatching = Input.reciprocalMatching;

	protected class Association {
		protected List<ClusterPoint> finalReferencePoints;
		protected List<ClusterPoint> finalTargetPoints;
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
			finalReferencePoints = new ArrayList<>();
			finalTargetPoints = new ArrayList<>();
			finalAssociations = new HashMap<>();

			for (Map.Entry<Integer, Integer> entry : reference.entrySet()) {
				Integer referenceIndex = entry.getKey();
				Integer targetIndex = entry.getValue();

				ClusterPoint referencePoint = originalReference.get(referenceIndex);
				ClusterPoint targetPoint = originalTarget.get(targetIndex);

				if (c_i.getJoint() != null) {
					double currentError;
					
					if(Input.distance.equals("Euclidean")){
						 currentError = referencePoint.getFPFH().squaredDistance(targetPoint.getFPFH());
					} else if(Input.distance.equals("ChiSquared")){
						currentError = referencePoint.getFPFH().chiSquare(targetPoint.getFPFH());
					} else {
						currentError = referencePoint.getFPFH().kullback(targetPoint.getFPFH());
					}
					
					totalError += currentError * Math.pow(c_i.getJoint().distance(referencePoint), 2);
				}

				if (target.containsKey(targetIndex)) {
					if ((reciprocalMatching && target.get(targetIndex) == referenceIndex) || !reciprocalMatching) {
						if (logging)
							IJ.log("Association between point nr. " + referenceIndex + " and point nr. " + targetIndex);
						finalReferencePoints.add(originalReference.get(referenceIndex));
						finalTargetPoints.add(originalTarget.get(targetIndex));
						finalAssociations.put(referenceIndex, targetIndex);
					}
				}
			}
		}
	}

	public ClosestPoint3(Cluster c_i, Cluster c_j) {
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		run();
	}

	private void run() {

		referencePoints = c_i.getPoints();
		targetPoints = c_j.getPoints();
		sourceAssociation = new HashMap<>();
		targetAssociation = new HashMap<>();

		ColorProcessor results;
		
		//TODO: Reject points with similar histograms to meanHistogram
		List<Histogram> referenceHistograms = c_i.getHistograms();
		Histogram meanReferenceHistogram = Histogram.meanHistogram(referenceHistograms);
		
		List<Histogram> targetHistograms = c_j.getHistograms();
		Histogram meanTargetHistogram = Histogram.meanHistogram(targetHistograms);
		
		
		
		//TODO
		

		sourceAssociation = getAssociation(referencePoints, targetPoints);
		targetAssociation = getAssociation(targetPoints, referencePoints);
		associations = getAssociatedPoints(sourceAssociation, targetAssociation);

		results = new ColorProcessor(Segmentation.width * 2, Segmentation.height);
		results.invert();

		if (Input.showAssociations) {
			Visualize.drawAssociations(results, associations.finalReferencePoints,
					Matrix.translate(associations.finalTargetPoints, 500, 0));
		}

		Visualize.drawPoints(results, referencePoints, Color.red);
		Visualize.drawPoints(results, Matrix.translate(targetPoints, 500, 0), Color.red);

		String fileName = "LRP_";
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
	 * method to get the closest point regarding the feature histogram for x'
	 * from a points set X'
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
			
			if(Input.distance.equals("Euclidean")){
				distanceNew = point.getFPFH().squaredDistance(referencePoints.get(i).getFPFH());
			} else if(Input.distance.equals("ChiSquared")){
				distanceNew = point.getFPFH().chiSquare(referencePoints.get(i).getFPFH());
			} else {
				distanceNew = point.getFPFH().kullback(referencePoints.get(i).getFPFH());
			}

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = i;
			}
		}
		return closestPoint;
	}

	public double getError() {
		return error;
	}

	private Association getAssociatedPoints(Map<Integer, Integer> reference, Map<Integer, Integer> target) {
		return new Association(reference, target, referencePoints, targetPoints);
	}

	public Map<Integer, Integer> getCorrespondences() {
		IJ.log("Associations found: " + associations.finalAssociations.size());
		return associations.finalAssociations;
	}
}
