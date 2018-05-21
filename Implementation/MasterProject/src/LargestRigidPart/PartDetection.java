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
	Cluster[] currentClusters = null;

	List<ClusterPoint> unclusteredReference = new ArrayList<>();
	List<ClusterPoint> unclusteredTarget = new ArrayList<>();

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
		int iteration = 1;

		List<ClusterPoint> currentLrpReference = null;
		List<ClusterPoint> currentLrpTarget = null;
		Cluster[] currentLrps = null;

		while (unclusteredReference.size() > MIN_SIZE || !clusters.isEmpty()) {
			IJ.log("Iteration #" + iteration++);
			
			if(iteration == 3){
				return;
			}

			removeAllLRPs();
			IJ.log("All LRPs removed from unclustered points");
			IJ.log("Unclustered Reference: " + unclusteredReference.size());
			IJ.log("Unclustered Target: " + unclusteredTarget.size());

			// grow regions from current lrp
			referenceClusters = RegionGrowing.detectClusters(unclusteredReference);
			targetClusters = RegionGrowing.detectClusters(unclusteredTarget);
			
			for(Cluster c : referenceClusters){
				ColorProcessor test = new ColorProcessor(Segmentation.width, Segmentation.height);
				test.invert();
				Visualize.drawPoints(test, c.getPoints(), Color.red);
				Visualize.showImage(test, "cluster detected");
			}

			IJ.log("Number of Reference Clusters: " + referenceClusters.size());
			IJ.log("Number of Target Clusters: " + referenceClusters.size());

			if (referenceClusters == null || targetClusters == null) {
				IJ.log("ERROR!");
			}

			if (currentClusters != null) {
				detectJoints(currentLrps, referenceClusters, targetClusters);
			}
			
			if(referenceClusters.size() != 0 || currentClusters == null){
				IJ.log("All clusters matched: " + clusters.size());
				pushMatchingClusters();
			}

			ColorProcessor results = new ColorProcessor(Segmentation.width, Segmentation.height);
			results.invert();

			if (currentLrpReference != null) {
				Visualize.drawPoints(results, currentLrpReference, Color.yellow);
				Visualize.drawPoints(results, currentLrpTarget, Color.green);
			}
			for (Cluster[] cluster : clusters) {
				if (cluster[0].getJoint() != null) {
					Visualize.drawDot(results, cluster[0].getJoint(), Color.red, 4);
					Visualize.drawDot(results, cluster[1].getJoint(), Color.blue, 4);
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
			ClosestPoint3 cp = new ClosestPoint3(currentClusters[0], currentClusters[1]);
			IJ.log("Finished ICP!");

			Map<Integer, Integer> denseCorrespondances = cp.getCorrespondences();
			
			if(denseCorrespondances.size() < 3){
				IJ.log("Too few correspondences detected!");
				return;
			}

			currentLrps = new LargestRigidPart(currentClusters[0], currentClusters[1], denseCorrespondances)
					.getLargestRigidParts();
			
			if (currentLrps[0].getPoints().size() < 15){
				IJ.log("No LRP found! --> Show largest cluster");
				return;
			}
			
			largestRigidParts.add(currentLrps);
			
			currentLrpReference = currentLrps[0].getPoints();
			currentLrpTarget = currentLrps[1].getPoints();
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
			double distanceNew = reference.getJoint().distance(target.getJoint());

			if (distanceNew < distance) {
				distance = distanceNew;
				matchingTarget = target;
			}
		}
		return matchingTarget;
	}

	private void detectJoints(Cluster[] currentLRPs, List<Cluster> referenceClusters, List<Cluster> targetClusters) {
		
		List<Cluster> copyClusters = new ArrayList<>();
		copyClusters.addAll(referenceClusters);
		
		for(Cluster cluster : copyClusters){
			cluster.setJoint(getJoint(cluster, currentLRPs[0]));
			if(cluster.getJoint() == null){
				referenceClusters.remove(cluster);
			}
		}
		
		copyClusters = new ArrayList<>();
		copyClusters.addAll(targetClusters);
		
		for(Cluster cluster : copyClusters){
			cluster.setJoint(getJoint(cluster, currentLRPs[1]));
			if(cluster.getJoint() == null){
				targetClusters.remove(cluster);
			}
		}
	}
	
	private ClusterPoint getJoint(Cluster c1, Cluster c2){
		double x = 0;
		double y = 0;
		double numberPoints = 0;
		
		for(ClusterPoint point1 : c1.getPoints()){
			for(ClusterPoint point2 : c2.getPoints()){
				if(point1.distance(point2) < Input.distanceThresholdRG){
					x += point1.getX() + point2.getX();
					y += point1.getY() + point2.getY();
					numberPoints+=2;
				}
			}
		}
		if(numberPoints == 0){
			IJ.log("No joint could be calculated!");
			return null;
		}
		return new ClusterPoint(x/numberPoints, y/numberPoints);
	}
}
