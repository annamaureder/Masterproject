package LargestRigidPart;

import java.util.List;

import ij.IJ;

public class NormalEstimation {

	ClusterPoint point;

	public NormalEstimation(ClusterPoint point) {
		this.point = point;
	}

	public void estimateNormal() {

		double avgX = 0.0;
		double sqrAvgX = 0.0;
		double avgY = 0.0;
		double sqrAvgY = 0.0;
		double avgXY = 0.0;

		double[][] covarianceMatrix;
		List<ClusterPoint> neighborhood = point.getNeighborhood();
		double size = neighborhood.size()* 1.0;
		
		for (int i = 0; i < neighborhood.size(); i++) {
			ClusterPoint current = neighborhood.get(i);
			
			avgX += current.getX();
			sqrAvgX += Math.pow(current.getX(), 2);
			avgY += current.getY();
			sqrAvgY += Math.pow(current.getY(), 2);
			avgXY += current.getX() * current.getY();
		}

		avgX /= size;
		sqrAvgX /= size;
		avgY /= size;
		sqrAvgY /= size;
		avgXY /= size;
		
		double a = sqrAvgX - avgX * avgX;
		double b = avgXY - avgX * avgY;
		double c = sqrAvgY - avgY * avgY ;
		
		//Solve equation for lambda (determinante)
				
		double lambda = (-1) * a - c;
		double f_a = 1;
		double f_b = lambda;
		double f_c = a * c - Math.pow(b, 2);

		//D = b^2 - 4ac
		double D = Math.pow(f_b, 2) - 4*f_a*f_c;
		double lambda1 = (-f_b -Math.sqrt(D))/(2*f_a);
		double lambda2 = (-f_b +Math.sqrt(D))/(2*f_a);
		double eigenvalue = Math.min(lambda1, lambda2);
		
		//use smallest lambda for normal vector
		
		covarianceMatrix = new double[][] { { a - eigenvalue, b},
			{ b, c - eigenvalue} };
			
		double[] normal = new double[2];
		
		normal[0] = -1.0;
		normal[1] = (eigenvalue * normal[0] - covarianceMatrix[0][0] * normal[0])/covarianceMatrix[0][1];
		
		if(Double.isInfinite(normal[1])){
			IJ.log("Infinite value!");
			normal[0] = 0;
			normal[1] = 1;
		} else if (Double.isNaN(normal[1])){
			normal[1] = 0;
		}
		
		//normalize vector to length 1
		
		double length = Math.sqrt(Math.pow(normal[0], 2) + Math.pow(normal[1], 2));
		
		normal[0] /= length;
		normal[1] /= length;
		
		point.setNormal(normal);
	}

}
