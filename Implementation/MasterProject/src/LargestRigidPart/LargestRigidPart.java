package LargestRigidPart;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import ij.IJ;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;
import prototyping.ICP;

/**
 * This plugin takes as input two point clouds c1 and c2 and returns the largest
 * rigid part by applying the RANSAC approach on the correspondacnes and by
 * getting the best affine transformation
 * 
 * 
 * @version 2013/08/22
 */
public class LargestRigidPart {

	// variable declaration

	private Cluster c_i;
	private Cluster c_j;

	List<double[]> points1;
	List<double[]> points2;
	
	List<double[]> randomPoints1;
	List<double[]> randomPoints2;

	private final int numIterations = 1000;
	private final int numRandom = 3;

	private double distanceThreshold = Input.distanceThresholdICP;

	private Cluster[] lrp;
	Cluster biggestClusterRef = new Cluster();
	Cluster biggestClusterTarget = new Cluster();
	
	private Map<Integer, Integer> correspondances;

	public LargestRigidPart(Cluster c_i, Cluster c_j, Map<Integer, Integer> correspondances) {

		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);
		this.correspondances = correspondances;

		performMatching();
	}

	private void performMatching() {
		points1 = c_i.getPoints();
		points2 = c_j.getPoints();

		for (int n = 0; n < numIterations; n++) {
			
			randomPoints1 = new ArrayList<>();
			randomPoints2 = new ArrayList<>();
			getRandomPoints(numRandom);

			// points from c1
			List<double[]> pointsA = new ArrayList<>();
			double[][] affineMatrix = fillTransformMatrix(pointsA);

			// points from c2
			double[] pointsB = new double[] { randomPoints2.get(0)[0], randomPoints2.get(0)[1], randomPoints2.get(1)[0],
					randomPoints2.get(1)[1], randomPoints2.get(2)[0], randomPoints2.get(2)[1] };

			DecompositionSolver solver = new SingularValueDecomposition(MatrixUtils.createRealMatrix(affineMatrix))
					.getSolver();
			RealVector affineVector = solver.solve(MatrixUtils.createRealVector(pointsB));

//			double[][] transformationMatrix = {
//					{ affineVector.getEntry(0), affineVector.getEntry(1), affineVector.getEntry(2) },
//					{ affineVector.getEntry(3), affineVector.getEntry(4), affineVector.getEntry(5) }, { 0, 0, 1 } };
//
//			for (int i = 0; i < transformationMatrix.length; i++) {
//				for (int j = 0; j < transformationMatrix[i].length; j++) {
//					IJ.log(" " + transformationMatrix[i][j]);
//				}
//				IJ.log("\n");
//			}

			List<double[]> resultPoints;
			double[] centroid = calculateCentroid(randomPoints1);

			resultPoints = Matrix.translate(points1, -centroid[0], -centroid[1]);
			resultPoints = Matrix.rotate(resultPoints, Math.acos(affineVector.getEntry(0)));
			resultPoints = Matrix.translate(resultPoints, centroid[0] + affineVector.getEntry(2),
					centroid[1] + affineVector.getEntry(5));

			Map<Integer, Integer> associations = getAssociation(resultPoints, points2);
			findBiggestCluster(associations);
		}
	}

	private double[][] fillTransformMatrix(List<double[]> vertices) {

		double[][] matrix = { { vertices.get(0)[0], vertices.get(0)[1], 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(0)[0], vertices.get(0)[1], 1 },
				{ vertices.get(1)[0], vertices.get(1)[1], 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(1)[0], vertices.get(1)[1], 1 },
				{ vertices.get(2)[0], vertices.get(2)[1], 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(2)[0], vertices.get(2)[1], 1 }, };

		return matrix;
	}

	private double[] calculateCentroid(List<double[]> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i)[0];
			avgY += points.get(i)[1];
		}

		return new double[] { avgX / points.size(), avgY / points.size() };
	}

	private final void findBiggestCluster(Map<Integer, Integer> associations) {
		List<double[]> ref = new ArrayList<>();
		List<double[]> target = new ArrayList<>();

		for (Map.Entry<Integer, Integer> entry : associations.entrySet()) {
			ref.add(points1.get(entry.getKey()));
			target.add(points2.get(entry.getValue()));
		}
		for (Cluster cluster : RegionGrowing.detectClusters(ref, null)) {
			if (cluster.getPoints().size() > biggestClusterRef.getPoints().size()) {
				biggestClusterRef = cluster;
			}
		}
		for (Cluster cluster : RegionGrowing.detectClusters(target, null)) {
			if (cluster.getPoints().size() > biggestClusterTarget.getPoints().size()) {
				biggestClusterTarget = cluster;
			}
		}
		lrp = new Cluster[] { biggestClusterRef, biggestClusterTarget };
	}

	private Map<Integer, Integer> getAssociation(List<double[]> originalPositions, List<double[]> targetPositions) {
		Map<Integer, Integer> associations = new HashMap<>();

		for (int i = 0; i < originalPositions.size(); i++) {
			int closestPoint = closestPoint(originalPositions.get(i), targetPositions);
			associations.put(i, closestPoint);
		}
		return associations;
	}

	private int closestPoint(double[] point, List<double[]> referencePoints) {
		int closestPoint = -1;
		double distance = Double.MAX_VALUE;
		double distanceNew = 0;

		for (int i = 0; i < referencePoints.size(); i++) {

			distanceNew = Math.pow(point[0] - referencePoints.get(i)[0], 2)
					+ Math.pow(point[1] - referencePoints.get(i)[1], 2);

			if (distanceNew < distance && Math.sqrt(distanceNew) < distanceThreshold) {
				distance = distanceNew;
				closestPoint = i;
			}
		}
		return closestPoint;
	}
	
	private void getRandomPoints(int num){
		int index = (int) (Math.random() * correspondances.size());
		Integer[] keys = correspondances.keySet().toArray(new Integer[0]);
		Integer[] values = correspondances.values().toArray(new Integer[0]);
		
		for(int i = 0; i < num; i++){
			randomPoints1.add(points1.get(keys[index]));
			randomPoints2.add(points2.get(values[index]));
		}
	}

	public Cluster[] getLargestRigidParts() {
		return lrp;
	}

}
