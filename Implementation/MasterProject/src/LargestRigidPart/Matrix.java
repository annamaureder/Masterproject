package LargestRigidPart;

import java.util.ArrayList;
import java.util.List;

public class Matrix {

	public static ClusterPoint multiplication(double[][] matrix, ClusterPoint point) {
		double[] vector = new double[] { point.getX(), point.getY(), 1 };
		double[] resultVector = new double[vector.length];

		for (int i = 0; i < matrix.length; i++) {
			double result = 0.0;
			for (int j = 0; j < matrix[0].length; j++) {
				result += matrix[i][j] * vector[j];
			}
			resultVector[i] = result;
		}
		return new ClusterPoint(resultVector[0] / resultVector[2], resultVector[1] / resultVector[2]);
	}

	public static List<ClusterPoint> translate(List<ClusterPoint> points, double translateX, double translateY) {
		List<ClusterPoint> translatedPoints = new ArrayList<>();
		double[][] T = new double[][] { { 1, 0, translateX }, { 0, 1, translateY }, { 0, 0, 1.0 } };

		for (int i = 0; i < points.size(); i++) {
			if(points.get(i)!= null){
				translatedPoints.add(Matrix.multiplication(T, points.get(i)));
			}else {
				translatedPoints.add(null);
			}
		}
		return translatedPoints;
	}

	public static List<ClusterPoint> rotate(List<ClusterPoint> points, double angle) {
		List<ClusterPoint> rotatedPoints = new ArrayList<>();

		double[][] R = new double[][] { { Math.cos(angle), -Math.sin(angle), 0 },
				{ Math.sin(angle), Math.cos(angle), 0 }, { 0, 0, 1 } };

		for (int i = 0; i < points.size(); i++) {
			rotatedPoints.add(Matrix.multiplication(R, points.get(i)));
		}
		return rotatedPoints;
	}
	
	public static List<ClusterPoint> rotate(List<ClusterPoint> points, double angle1, double angle2, double angle3, double angle4) {
		List<ClusterPoint> rotatedPoints = new ArrayList<>();

		double[][] R = new double[][] { { angle1, angle2, 0 },
				{ angle3, angle4, 0 }, { 0, 0, 1 } };

		for (int i = 0; i < points.size(); i++) {
			rotatedPoints.add(Matrix.multiplication(R, points.get(i)));
		}
		return rotatedPoints;
	}
}
