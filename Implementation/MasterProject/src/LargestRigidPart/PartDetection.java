package LargestRigidPart;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import ij.IJ;
import ij.process.ColorProcessor;

/**
 * This plugin takes as input two point clouds c1 and c2 and returns the largest
 * rigid part by applying the ICP and a RANSAC approach with Procrustes
 * fitting.Ï
 * 
 * @version 2013/08/22
 */
public class PartDetection {

	// variable declaration

	private Cluster c_i;
	private Cluster c_j;

	private final int MIN_SIZE = 10;

	List<double[]> unclusteredReference = new ArrayList<>();
	List<double[]> unclusteredTarget = new ArrayList<>();

	// cluster pairs of each a reference and target cluster
	private Stack<Cluster[]> clusters = new Stack<>();
	List<Cluster> referenceClusters = new ArrayList<>();
	List<Cluster> targetClusters = new ArrayList<>();

	private List<Cluster[]> largestRigidParts = new ArrayList<>();

	public PartDetection(Cluster c_i, Cluster c_j) {
		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		unclusteredReference = this.c_i.getPoints();
		unclusteredTarget = this.c_j.getPoints();

		run();
	}

	private void run() {
		Cluster currentLrpReference = null;
		Cluster currentLrpTarget = null;
		int iteration = 1;

		Cluster[] currentClusters = null;
		Cluster[] currentLrps = null;

		while (unclusteredReference.size() > MIN_SIZE || clusters.isEmpty()) {

			if (iteration == 5) {
				return;
			}

			IJ.log("Iteration #" + iteration++);

			removeAllLRPs();
			IJ.log("All LRPs removed from unclustered points");
			IJ.log("Unclustered Reference: " + unclusteredReference.size());
			IJ.log("Unclustered Target: " + unclusteredTarget.size());

			// grow regions from current lrp

			referenceClusters = RegionGrowing.detectClusters(unclusteredReference);
			targetClusters = RegionGrowing.detectClusters(unclusteredTarget);

			IJ.log("Number of Reference Clusters: " + referenceClusters.size());
			IJ.log("Number of Target Clusters: " + referenceClusters.size());

			if (referenceClusters == null || targetClusters == null) {
				IJ.log("ERROR!");
			}

			if (currentClusters != null) {
				detectJoints(currentLrps, referenceClusters, targetClusters);
			}

			pushMatchingClusters();
			IJ.log("All clusters matched: " + clusters.size());

			ColorProcessor results = new ColorProcessor(Segmentation.width, Segmentation.height);
			results.invert();

			if (currentLrpReference != null) {
				Visualize.drawPoints(results, currentLrpReference.getPoints(), Color.yellow);
				Visualize.drawPoints(results, currentLrpTarget.getPoints(), Color.green);
			}
			for (Cluster[] cluster : clusters) {
				if (cluster[0].getJoint() != null) {
					Visualize.drawDot(results, cluster[0].getJoint(), Color.red);
					Visualize.drawDot(results, cluster[1].getJoint(), Color.blue);
				}
				Visualize.drawPoints(results, cluster[0].getPoints(), Color.red);
				Visualize.drawPoints(results, cluster[1].getPoints(), Color.blue);
			}
			
			Visualize.showImage(results, "Matching clusters");

			if (clusters.isEmpty()) {
				IJ.log("Empty stack!");
				return;
			}

			currentClusters = clusters.pop();
			ClosestPoint cp = new ClosestPoint(currentClusters[0], currentClusters[1]);
			IJ.log("Finished ICP!");

			Map<Integer, Integer> denseCorrespondances = cp.getCorrespondences();

			currentLrps = new LargestRigidPart(currentClusters[0], currentClusters[1], denseCorrespondances)
					.getLargestRigidParts();
			largestRigidParts.add(currentLrps);
		}
	}

	public List<Cluster[]> getRigidParts() {
		return largestRigidParts;
	}

	private void removeAllLRPs() {
		for (Cluster[] lrp : largestRigidParts) {
			unclusteredReference.removeAll(lrp[0].getPoints());
			unclusteredTarget.removeAll(lrp[1].getPoints());
		}
	}

	private void pushMatchingClusters() {
		for (Cluster reference : referenceClusters) {
			if (referenceClusters.size() == 1 || targetClusters.size() == 1) {
				clusters.push(new Cluster[] { reference, targetClusters.get(0) });
			} else {
				Cluster target = matchingTarget(reference);
				clusters.push(new Cluster[] { reference, target });
			}
		}
	}

	private Cluster matchingTarget(Cluster reference) {
		Cluster matchingTarget = null;
		double distance = Double.MAX_VALUE;

		for (Cluster target : targetClusters) {
			double distanceNew = Math.pow(reference.getJoint()[0] - target.getJoint()[0], 2)
					+ Math.pow(reference.getJoint()[1] - target.getJoint()[1], 2);

			if (distanceNew < distance) {
				distance = distanceNew;
				matchingTarget = target;
			}
		}
		return matchingTarget;
	}

	private void detectJoints(Cluster[] currentLRPs, List<Cluster> referenceClusters, List<Cluster> targetClusters) {
		
		for(Cluster cluster : referenceClusters){
			cluster.setJoint(getJoint(cluster, currentLRPs[0]));
		}
		
		for(Cluster cluster : targetClusters){
			cluster.setJoint(getJoint(cluster, currentLRPs[1]));
		}
	}
	
	private double[] getJoint(Cluster c1, Cluster c2){
		double x = 0;
		double y = 0;
		double numberPoints = 0;
		
		for(double[] point1 : c1.getPoints()){
			for(double[] point2 : c2.getPoints()){
				if(distance(point1, point2) < Input.distanceThresholdRG){
					x += point1[0] + point2[0];
					y += point1[1] + point2[1];
					numberPoints+=2;
				}
			}
		}
		if(numberPoints == 0){
			IJ.log("No joints could be calculated!");
			return null;
		}
		return new double[]{x/numberPoints, y/numberPoints};
	}
	
	private double distance(double[] point1, double[] point2) {
		return Math.sqrt(Math.pow(point1[0]-point2[0], 2) + Math.pow(point1[1]-point2[1], 2));
	}
}
