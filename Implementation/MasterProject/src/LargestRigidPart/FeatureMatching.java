package LargestRigidPart;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
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
public class FeatureMatching {

	private Cluster c_i;
	private Cluster c_j;

	private double error = Double.MAX_VALUE;
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

			File file = new File("../src/LargestRigidPart/histogram.txt");
			FileWriter fileWriter = null;
			try {
				fileWriter = new FileWriter(file);
			} catch (IOException e) {
				e.printStackTrace();
			}

			for (Map.Entry<Integer, Integer> entry : reference.entrySet()) {
				Integer referenceIndex = entry.getKey();
				Integer targetIndex = entry.getValue();

				ClusterPoint currentRefPoint = originalReference.get(referenceIndex);
				ClusterPoint currentTargetPoint = originalTarget.get(targetIndex);

				// joint weights in case of reapplying recursively
				
//				if (c_i.getJoint() != null) {
//					double currentError;
//
//					if (Input.distance.equals("Euclidean")) {
//						currentError = currentRefPoint.getFPFH().squaredDistance(currentTargetPoint.getFPFH());
//					} else if (Input.distance.equals("ChiSquared")) {
//						currentError = currentRefPoint.getFPFH().chiSquare(currentTargetPoint.getFPFH());
//					} else {
//						currentError = currentRefPoint.getFPFH().kullback(currentTargetPoint.getFPFH());
//					}
//					totalError += currentError * Math.pow(c_i.getJoint().distance(currentTargetPoint), 2);
//				}

				if (target.containsKey(targetIndex)) {
					if ((reciprocalMatching && target.get(targetIndex) == referenceIndex) || !reciprocalMatching) {

						finalReferencePoints.add(currentRefPoint);
						finalTargetPoints.add(currentTargetPoint);
						finalAssociations.put(referenceIndex, targetIndex);

						try {
							fileWriter.write(currentRefPoint.getFPFH().toString() + ", " + currentTargetPoint.getFPFH().toString() + "\n");
						} catch (IOException e) {
							e.printStackTrace();
						}
						try {
							fileWriter.flush();
						} catch (IOException e) {
							e.printStackTrace();
						}

					}
				}
			}
		}
	}

	public FeatureMatching(Cluster c_i, Cluster c_j) {
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

		// TODO: Reject points with similar histograms to meanHistogram
		List<Histogram> referenceHistograms = c_i.getHistograms();
		Histogram meanReferenceHistogram = Histogram.meanHistogram(referenceHistograms);

		IJ.log("Number of reference Histograms: " + referenceHistograms.size());
		IJ.log("Mean histogram: ");

		IJ.log("Standard deviation: " + meanReferenceHistogram.getStandardDeviation());

		List<Histogram> targetHistograms = c_j.getHistograms();
		Histogram meanTargetHistogram = Histogram.meanHistogram(targetHistograms);
		IJ.log("Standard deviation: " + meanTargetHistogram.getStandardDeviation());

		// referencePoints = new ArrayList<>();
		// targetPoints = new ArrayList<>();
		//
		// for (ClusterPoint point : c_i.getPoints()){
		// Histogram current = point.getFPFH();
		//
		// for(int i = 0; i < current.getHistogram().length; i++){
		// if(Math.abs(current.getHistogram()[i] -
		// meanTargetHistogram.getHistogram()[i]) > 1){
		// referencePoints.add(point);
		// break;
		// }
		// }
		// }
		//
		// for (ClusterPoint point : c_j.getPoints()){
		// Histogram current = point.getFPFH();
		//
		// for(int i = 0; i < current.getHistogram().length; i++){
		// if(Math.abs(current.getHistogram()[i] -
		// meanReferenceHistogram.getHistogram()[i]) > 1){
		// targetPoints.add(point);
		// break;
		// }
		// }
		// }

		IJ.log(referencePoints.size() + " points selected for feature matching (reference).");
		IJ.log(targetPoints.size() + " points selected for feature matching (target).");

		// TODO
		sourceAssociation = getAssociation(referencePoints, targetPoints);
		targetAssociation = getAssociation(targetPoints, referencePoints);
		associations = getAssociatedPoints(sourceAssociation, targetAssociation);

		results = new ColorProcessor(Main.width * 2, Main.height);
		results.invert();

		if (Input.showAssociations) {
			Visualize.drawAssociations(results, associations.finalReferencePoints,
					Matrix.translate(associations.finalTargetPoints, 500, 0));
		}

		Visualize.drawPoints(results, c_i.getPoints(), Color.red);
		Visualize.drawPoints(results, Matrix.translate(c_j.getPoints(), 500, 0), Color.blue);

		String fileName = "LRP_";
		if (reciprocalMatching) {
			fileName += "_reciprocal";
		}
		Visualize.addToResults(results, fileName);
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
	 * @param p_i
	 * @param c_2
	 * @return
	 */

	private int closestPoint(ClusterPoint p_i, List<ClusterPoint> c_2) {
		int closestPoint = -1;
		double distance = Double.MAX_VALUE;
		double distanceNew = 0;

		for (int i = 0; i < c_2.size(); i++) {

			if (Input.distance.equals("Euclidean")) {
				distanceNew = p_i.getFPFH().squaredDistance(c_2.get(i).getFPFH());
			} else if (Input.distance.equals("ChiSquared")) {
				distanceNew = p_i.getFPFH().chiSquare(c_2.get(i).getFPFH());
			} else {
				distanceNew = p_i.getFPFH().kullback(c_2.get(i).getFPFH());
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
