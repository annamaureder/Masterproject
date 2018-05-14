package LargestRigidPart;

import java.util.List;

public class NormalEstimation {

	ClusterPoint point;

	public NormalEstimation(ClusterPoint point) {
		this.point = point;
	}

	public void estimateNormal() {

		double avgX = point.getX();
		double sqrAvgX = Math.pow(point.getX(), 2);
		double avgY = point.getY();
		double sqrAvgY = Math.pow(point.getY(), 2);
		double avgXY = point.getX() * point.getY();

		double[][] covarianceMatrix;
		List<ClusterPoint> neighborhood = point.getNeighborhood();

		for (int i = 0; i < neighborhood.size(); i++) {
			ClusterPoint current = neighborhood.get(i);

			avgX += current.getX();
			sqrAvgX += Math.pow(current.getX(), 2);
			avgY += current.getY();
			sqrAvgY += Math.pow(current.getY(), 2);
			avgXY += current.getX() * current.getY();
		}

		avgX /= neighborhood.size();
		sqrAvgX /= neighborhood.size();
		avgY /= neighborhood.size();
		sqrAvgY /= neighborhood.size();
		avgXY /= neighborhood.size();

		covarianceMatrix = new double[][] { { sqrAvgX - avgX * avgX, avgXY - avgX * avgY },
				{ avgXY - avgX * avgY, sqrAvgY - avgY * avgY } };
				
		double[] ab = null;
		double lamda = 0;
		
		//Solver: covarianceMatrix * ab = lamda * ab
		//select eigenvector with smalles eigenvalue as normal n

		point.setNormal(ab);
		
	}

}
