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
import ij.process.ImageProcessor;
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
public class RANSAC {

	// variable declaration
	private boolean logging = Input.logging;

	private Cluster c_i;
	private Cluster c_j;

	List<ClusterPoint> points1;
	List<ClusterPoint> points2;

	List<ClusterPoint> randomPoints1;
	List<ClusterPoint> randomPoints2;

	private final int numIterations = 1000;
	private final int numRandom = 3;
	List<ClusterPoint> resultPoints;

	private double distanceThreshold = Input.distanceThresholdRANSAC;

	private Cluster[] lrp;
	Cluster biggestClusterRef = new Cluster();
	Cluster biggestClusterTarget = new Cluster();
	
	List<ClusterPoint> finalRef = new ArrayList<>();
	List<ClusterPoint> finalTar = new ArrayList<>();

	private Map<Integer, Integer> correspondances;

	public RANSAC(Cluster c_i, Cluster c_j, Map<Integer, Integer> correspondances) {

		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);
		this.correspondances = correspondances;

		performMatching();
	}

	private void performMatching() {
		points1 = c_i.getPoints();
		points2 = c_j.getPoints();

		for (int n = 0; n < numIterations; n++) {
			if (logging)
				IJ.log("RANSAC iteration #" + n);

			randomPoints1 = new ArrayList<>();
			randomPoints2 = new ArrayList<>();
			getRandomPoints(numRandom);

			if (logging)
				IJ.log("Random points calculated");

			// points from c1
			List<ClusterPoint> pointsA = randomPoints1;
			double[][] affineMatrix = fillTransformMatrix(pointsA);

			if (logging)
				IJ.log("affine Matrix filled!");

			// points from c2
			double[] pointsB = new double[] { randomPoints2.get(0).getX(), randomPoints2.get(0).getY(), randomPoints2.get(1).getX(),
					randomPoints2.get(1).getY(), randomPoints2.get(2).getX(), randomPoints2.get(2).getY() };

			if (logging)
				IJ.log("points B filed!");

			DecompositionSolver solver = new SingularValueDecomposition(MatrixUtils.createRealMatrix(affineMatrix))
					.getSolver();
			RealVector affineTransformation = solver.solve(MatrixUtils.createRealVector(pointsB));

			if (logging)
				IJ.log("Transformation Matrix:");

			double[][] transformationMatrix = {
					{ affineTransformation.getEntry(0), affineTransformation.getEntry(1),
							affineTransformation.getEntry(2) },
					{ affineTransformation.getEntry(3), affineTransformation.getEntry(4),
							affineTransformation.getEntry(5) },
					{ 0, 0, 1 } };

			for (int i = 0; i < transformationMatrix.length; i++) {
				for (int j = 0; j < transformationMatrix[i].length; j++) {
					if (logging)
						IJ.log(" " + transformationMatrix[i][j]);
				}
				if (logging)
					IJ.log("\n");
			}

			double[] centroid = calculateCentroid(randomPoints1);

			if (logging)
				IJ.log("Centroid calculated!");

			resultPoints = Matrix.translate(points1, -centroid[0], -centroid[1]);
			resultPoints = Matrix.rotate(resultPoints, affineTransformation.getEntry(0), affineTransformation.getEntry(1), affineTransformation.getEntry(3), affineTransformation.getEntry(4));
			resultPoints = Matrix.translate(resultPoints, centroid[0] + affineTransformation.getEntry(2),
					centroid[1] + affineTransformation.getEntry(5));

			if (logging)
				IJ.log("Transformation of points done!");

			Map<Integer, Integer> associations = getAssociation(resultPoints, points2);

			if (logging)
				IJ.log("associations found: " + associations.size());

			findBiggestCluster(associations);

			if (logging)
				IJ.log("Cluster detected!");
		}

		if (logging)
			IJ.log("Final LRP found!");
		ColorProcessor resultReference = new ColorProcessor(Main.width, Main.height);
		resultReference.invert();
		
		ColorProcessor resultTarget = new ColorProcessor(Main.width, Main.height);
		resultTarget.invert();
		
		ColorProcessor bestTransformation = new ColorProcessor(Main.width, Main.height);
		bestTransformation.invert();
		
		Visualize.drawPoints(bestTransformation, finalRef, Color.blue);
		Visualize.drawPoints(bestTransformation, finalTar, Color.red);
		
		Visualize.showImage(bestTransformation, "Best transformation reference");
		
		Visualize.drawPoints(resultReference, biggestClusterRef.getPoints(), Color.blue);
		Visualize.drawPoints(resultTarget, biggestClusterTarget.getPoints(), Color.red);
		
		Visualize.showImage(resultReference, "Final LRP reference");
		Visualize.showImage(resultTarget, "Final LRP target");
		
		List<ClusterPoint> list1 = new ArrayList<>();
		List<ClusterPoint> list2 = new ArrayList<>();
		
		for (Map.Entry<Integer, Integer> entry : correspondances.entrySet()) {
			list1.add(c_i.getPoints().get(entry.getKey()));
			list2.add(c_j.getPoints().get(entry.getValue()));
		}
	
	}

	private double[][] fillTransformMatrix(List<ClusterPoint> vertices) {
		double[][] matrix = { { vertices.get(0).getX(), vertices.get(0).getY(), 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(0).getX(), vertices.get(0).getY(), 1 },
				{ vertices.get(1).getX(), vertices.get(1).getY(), 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(1).getX(), vertices.get(1).getY(), 1 },
				{ vertices.get(2).getX(), vertices.get(2).getY(), 1, 0, 0, 0 },
				{ 0, 0, 0, vertices.get(2).getX(), vertices.get(2).getY(), 1 }, };

		return matrix;
	}

	private double[] calculateCentroid(List<ClusterPoint> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i).getX();
			avgY += points.get(i).getY();
		}
		return new double[] { avgX / points.size(), avgY / points.size() };
	}

	private final void findBiggestCluster(Map<Integer, Integer> associations) {
		List<ClusterPoint> ref = new ArrayList<>();
		List<ClusterPoint> target = new ArrayList<>();

		for (Map.Entry<Integer, Integer> entry : associations.entrySet()) {
			ref.add(points1.get(entry.getKey()));
			target.add(points2.get(entry.getValue()));
		}
		
		for (Cluster cluster : RegionGrowing.detectClusters(ref)) {
			if (cluster.getPoints().size() > biggestClusterRef.getPoints().size()) {
				biggestClusterRef = cluster;
				finalRef = resultPoints;
			}
		}
		for (Cluster cluster : RegionGrowing.detectClusters(target)) {
			if (cluster.getPoints().size() > biggestClusterTarget.getPoints().size()) {
				biggestClusterTarget = cluster;
				finalTar = points2;
			}
		}
		lrp = new Cluster[] { biggestClusterRef, biggestClusterTarget };
	}

	private Map<Integer, Integer> getAssociation(List<ClusterPoint> originalPositions, List<ClusterPoint> targetPositions) {
		Map<Integer, Integer> associations = new HashMap<>();

		for (int i = 0; i < originalPositions.size(); i++) {
			int closestPoint = closestPoint(originalPositions.get(i), targetPositions);
			if (closestPoint != -1) {
				associations.put(i, closestPoint);
			}
		}
		
		return associations;
	}

	private int closestPoint(ClusterPoint point, List<ClusterPoint> referencePoints) {
		int closestPoint = -1;
		double distance = Double.MAX_VALUE;
		double distanceNew = 0;

		for (int i = 0; i < referencePoints.size(); i++) {
			distanceNew = point.distance(referencePoints.get(i));
			
			if (distanceNew < distance && distanceNew < distanceThreshold) {
				distance = distanceNew;
				closestPoint = i;
			}
		}
		return closestPoint;
	}

	private void getRandomPoints(int num) {
		int index;

		if (logging)
			IJ.log("Number of correspondances: " + correspondances.size());
		Integer[] keys = correspondances.keySet().toArray(new Integer[0]);
		Integer[] values = correspondances.values().toArray(new Integer[0]);

		for (int i = 0; i < num; i++) {
			index = (int) (Math.random() * correspondances.size());
			randomPoints1.add(points1.get(keys[index]));
			randomPoints2.add(points2.get(values[index]));
			if (logging)
				IJ.log("Random index: " + index);
		}
	}

	public Cluster[] getLargestRigidParts() {
		return lrp;
	}
}
