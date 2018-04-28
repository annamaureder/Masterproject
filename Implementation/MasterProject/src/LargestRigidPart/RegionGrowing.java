package LargestRigidPart;

import java.util.ArrayList;
import java.util.List;

public class RegionGrowing {
	
	//ToDo: Check algorithm
	
	private static double distanceThreshold = Input.distanceThreshold;

	/**
	 * This method detects all clusters of the input points, optionally taking
	 * the seeds as starting point in the region growing.
	 * The output is a list of all detected clusters including their initial seeds.
	 * 
	 * @param allPoints
	 *            Input points
	 * @param seedPoints
	 *            Seed points
	 * @return
	 */
	public static List<Cluster> detectClusters(List<double[]> allPoints, List<double[]> seedPoints) {
		if (seedPoints == null) {
			seedPoints = allPoints;
		}

		List<double[]> current = new ArrayList<>();
		List<Cluster> clusters = new ArrayList<>();
		double[] seed;

		while (!allPoints.isEmpty()) {
			seed = seedPoints.get(0);
			current.add(seed);

			for (int c = 0; c < current.size(); c++) {
				allPoints.removeAll(current);
				for (int i = 0; i < allPoints.size(); i++) {
					if (distance(current.get(c), allPoints.get(i)) < distanceThreshold) {
						current.add(allPoints.get(i));
					}
				}
			}
			allPoints.removeAll(current);
			Cluster c = new Cluster(current);
			c.setJoint(seed);

			clusters.add(c);

			current = new ArrayList<double[]>();
		}
		return clusters;
	}

	private static double distance(double[] current, double[] point) {
		double x = Math.abs(current[0] - point[0]);
		double y = Math.abs(current[1] - point[1]);

		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}
}
