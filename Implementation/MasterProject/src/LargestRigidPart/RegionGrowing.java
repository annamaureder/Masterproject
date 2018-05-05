package LargestRigidPart;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ij.IJ;

public class RegionGrowing {

	private static double distanceThreshold = Input.distanceThresholdRG;
	private final static int MIN_SIZE = 5;

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
	public static List<Cluster> detectClusters(List<double[]> allPoints) {

		List<double[]> inputPoints = new ArrayList<>();
		inputPoints.addAll(allPoints);

		List<double[]> current = new ArrayList<>();
		List<Cluster> clusters = new ArrayList<>();
		double[] seed;

		while (!inputPoints.isEmpty()) {
			seed = inputPoints.get(0);
			current.add(seed);

			for (int c = 0; c < current.size(); c++) {
				inputPoints.removeAll(current);
				for (int i = 0; i < inputPoints.size(); i++) {
					if (distance(current.get(c), inputPoints.get(i)) < distanceThreshold) {
						current.add(inputPoints.get(i));
					}
				}
			}
			inputPoints.removeAll(current);

			if (current.size() > MIN_SIZE) {
				Cluster c = new Cluster(current);
				clusters.add(c);
			}
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
