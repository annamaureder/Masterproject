package LargestRigidPart;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;
import ij.IJ;

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

	private List<double[]> denseCorrespondances_c1;
	private List<double[]> denseCorrespondances_c2;

	private int minSize = 10;

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

		unclusteredReference = c_i.getPoints();
		unclusteredTarget = c_j.getPoints();

		run();
	}

	private void run() {
		Cluster currentLrpReference = null;
		Cluster currentLrpTarget = null;

		while (c_i.getPoints().size() > minSize || clusters.isEmpty()) {

			removeAllLRPs();
			IJ.log("All LRPs removed from unclustered points");

			// grow regions from current lrp

			if(currentLrpReference == null || currentLrpTarget == null){
				referenceClusters = RegionGrowing.detectClusters(unclusteredReference, null);
				targetClusters = RegionGrowing.detectClusters(unclusteredTarget, null);
			}
			else{
				referenceClusters = RegionGrowing.detectClusters(unclusteredReference, currentLrpReference.getPoints());
				targetClusters = RegionGrowing.detectClusters(unclusteredTarget, currentLrpTarget.getPoints());
			}
			
			if(referenceClusters == null || targetClusters == null){
				IJ.log("ERROR!");
			}
			
			pushMatchingClusters();
			IJ.log("All clusters matched");

			// get current lrp

			Cluster[] currentClusters = clusters.pop();
			ClosestPoint cp = new ClosestPoint(currentClusters[0], currentClusters[1]);

			denseCorrespondances_c1 = cp.getReferencePoints();
			denseCorrespondances_c2 = cp.getTargetPoints();

			IJ.log("Finished!");

			// ToDo: get rigid part that is either linked to a joint OR largestRigidPart
			
//			List<List<double[]>> denseCorrespondances = new ArrayList<>();
//			denseCorrespondances.add(denseCorrespondances_c1);
//			denseCorrespondances.add(denseCorrespondances_c2);
//
//			Cluster[] currentLrps = new LargestRigidPart(c_i, c_j, denseCorrespondances).getClusters();
//			currentLrpReference = currentLrps[0];
//			currentLrpTarget = currentLrps[1];
//			
//			largestRigidParts.add(currentLrps);
		}

	}

	public List<Cluster[]> getRigidParts() {
		return largestRigidParts;
	}

	private void removeAllLRPs() {
		for (Cluster[] lrp : largestRigidParts) {
			unclusteredReference.remove(lrp[0].getPoints());
			unclusteredTarget.remove(lrp[1].getPoints());
		}

	}
	
	private void pushMatchingClusters(){
		for(Cluster reference : referenceClusters){
			if(referenceClusters.size() == 1 || targetClusters.size() == 1){
				clusters.push(new Cluster[]{reference, targetClusters.get(0)});
			}
			else{
				Cluster target = matchingTarget(reference);
				clusters.push(new Cluster[]{reference, target});
			}
			
			IJ.log("Cluster size: " + clusters.size());;
		}
	}
	
	private Cluster matchingTarget(Cluster reference){
		Cluster matchingTarget = null;
		double distance = Double.MAX_VALUE;
		
		for(Cluster target : targetClusters){
			double distanceNew = Math.pow(reference.getJoint()[0] - target.getJoint()[0],2) + Math.pow(reference.getJoint()[1] - target.getJoint()[1],2);
			
			if(distanceNew < distance){
				distance = distanceNew;
				matchingTarget = target;
			}
		}
		return matchingTarget;
	}
}
