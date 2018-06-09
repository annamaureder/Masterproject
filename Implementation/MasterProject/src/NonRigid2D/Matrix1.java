package NonRigid2D;

import java.util.ArrayList;
import java.util.List;

import LargestRigidPart.ClusterPoint;
import LargestRigidPart.Matrix;

public class Matrix1 {

	public static ClusterPoint1 multiplication(double[][] matrix, ClusterPoint1 point) {
		double[] vector = new double[] { point.getX(), point.getY(), 1 };
		double[] resultVector = new double[vector.length];

		for (int i = 0; i < matrix.length; i++) {
			double result = 0.0;
			for (int j = 0; j < matrix[0].length; j++) {
				result += matrix[i][j] * vector[j];
			}
			resultVector[i] = result;
		}
		return new ClusterPoint1(resultVector[0] / resultVector[2], resultVector[1] / resultVector[2]);
	}

	public static List<ClusterPoint1> translate(List<ClusterPoint1> points, double translateX, double translateY) {
		List<ClusterPoint1> translatedPoints = new ArrayList<>();
		double[][] T = new double[][] { { 1, 0, translateX }, { 0, 1, translateY }, { 0, 0, 1.0 } };

		for (int i = 0; i < points.size(); i++) {
			if(points.get(i)!= null){
				translatedPoints.add(Matrix1.multiplication(T, points.get(i)));
			}else {
				translatedPoints.add(null);
			}
		}
		return translatedPoints;
	}

	public static List<ClusterPoint1> rotate(List<ClusterPoint1> points, double angle) {
		List<ClusterPoint1> rotatedPoints = new ArrayList<>();

		double[][] R = new double[][] { { Math.cos(angle), -Math.sin(angle), 0 },
				{ Math.sin(angle), Math.cos(angle), 0 }, { 0, 0, 1 } };

		for (int i = 0; i < points.size(); i++) {
			rotatedPoints.add(Matrix1.multiplication(R, points.get(i)));
		}
		return rotatedPoints;
	}
	
	public static List<ClusterPoint1> rotate(List<ClusterPoint1> points, double angle1, double angle2, double angle3, double angle4) {
		List<ClusterPoint1> rotatedPoints = new ArrayList<>();

		double[][] R = new double[][] { { angle1, angle2, 0 },
				{ angle3, angle4, 0 }, { 0, 0, 1 } };

		for (int i = 0; i < points.size(); i++) {
			rotatedPoints.add(Matrix1.multiplication(R, points.get(i)));
		}
		return rotatedPoints;
	}
}
