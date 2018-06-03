package LargestRigidPart;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ij.IJ;

public class RegionGrowing {

	private static double distanceThreshold = Input.distanceThresholdRG;
	private static int MIN_SIZE = 5;

	/**
	 * This method detects all clusters of the input points, optionally taking
	 * the seeds as starting point in the region growing. The output is a list
	 * of all detected clusters including their initial seeds.
	 * 
	 * @param allPoints
	 *            Input points
	 * @param seedPoints
	 *            Seed points
	 * @return
	 */
	public static List<Cluster> detectClusters(List<ClusterPoint> allPoints, List<ClusterPoint> seedPoints, int maxPoints) {
		
		List<ClusterPoint> seeds = new ArrayList<>();
		List<ClusterPoint> inputPoints = new ArrayList<>();

		inputPoints.addAll(allPoints);

		if (seedPoints == null) {
			seeds.addAll(allPoints);
		} else {
			seeds.addAll(seedPoints);	
		}
		
		if(maxPoints == 0){
			maxPoints = allPoints.size();
		}
		else{
			MIN_SIZE = maxPoints;
		}
		
		List<ClusterPoint> current = new ArrayList<>();
		List<Cluster> clusters = new ArrayList<>();
		ClusterPoint seed;

		while (!seeds.isEmpty() && current.size() < maxPoints) {
			seed = seeds.get(0);
			current.add(seed);
			seeds.remove(seed);
			

			for (int c = 0; c < current.size(); c++) {
				inputPoints.removeAll(current);
				seeds.removeAll(current);
				
				for (ClusterPoint point : inputPoints) {
					if (current.get(c).distance(point) < distanceThreshold) {
						if(current.size() < maxPoints){
							current.add(point);
						}
						else{
							break;
						}
					}					
				}
			}
			inputPoints.removeAll(current);
			seeds.removeAll(current);
			
			if (current.size() >= MIN_SIZE) {
				Cluster c = new Cluster(current);
				clusters.add(c);
			}
			
			current = new ArrayList<>();
		}
		return clusters;
	}
	
	public static List<Cluster> detectClusters(List<ClusterPoint> allPoints) {
		return detectClusters(allPoints, null, 0);
	}
	
	public static List<Cluster> nearestNeighbors(List<ClusterPoint> allPoints, ClusterPoint point, int maxPoints) {
		List<ClusterPoint> singlePointList = new ArrayList<>();
		singlePointList.add(point);
		return detectClusters(allPoints, singlePointList, maxPoints + 1);
	}
}
