package NonRigid2D;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import LargestRigidPart.ClusterPoint;
import ij.IJ;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;

/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class Registration1 {

	// variable declaration
	private Cluster1 c_i;
	private Cluster1 c_j;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;
	private double errorPerPoint = Double.MAX_VALUE;
	private double errorThreshold = Input1.errorThreshold;

	private List<ClusterPoint1> finalTargetPoints;
	private List<ClusterPoint1> finalReferencePoints;

	private List<ClusterPoint1> association;
	private int amountPoints;

	public Registration1(Cluster1 c_i, Cluster1 c_j) {

		this.c_i = new Cluster1(c_i);
		this.c_j = new Cluster1(c_j);

		amountPoints = c_i.getPoints().size();
		run();
	}

	private void run() {
		if (amountPoints < 2) {
			return;
		}

		//Step 1: initial alignment
		
		c_i.alignAxis();
		c_j.alignAxis();
		
		List<ClusterPoint1> referencePoints = Matrix1.translate(c_i.getPoints(),
				c_j.getCentroid().getX() - c_i.getCentroid().getX(),
				c_j.getCentroid().getY() - c_i.getCentroid().getY());
		List<ClusterPoint1> targetPoints = c_j.getPoints();
		
		ColorProcessor initialAlignment = new ColorProcessor(Segmentation1.width, Segmentation1.height); 
		initialAlignment.invert();
		Visualize1.drawPoints(initialAlignment, referencePoints, Color.black);
		Visualize1.drawPoints(initialAlignment, targetPoints, Color.red);
		association = getAssociation(referencePoints, targetPoints);
		Visualize1.drawAssociations(initialAlignment, referencePoints, association);

		Visualize1.addToResults(initialAlignment, "Initial alignment");

		/*
		 * Step 2 - associate Points
		 */
		association = new ArrayList<>();
		ProcrustesFit pro = new ProcrustesFit();

		int iterations = 0;

		do {
			tmp_error = 0;
			association = getAssociation(referencePoints, targetPoints);
			
			// calculate transformation between points1 and points2
			pro.fit(getPoints(referencePoints), getPoints(association));

			if (tmp_error < error) {
				error = tmp_error;
				finalTargetPoints = association;
				finalReferencePoints = referencePoints;
			}
			
			else{
				return;
			}
			
			errorPerPoint = Math.sqrt(error / amountPoints);
			IJ.log("error per point: " + errorPerPoint);
						
			ClusterPoint1 c = calculateCentroid(referencePoints);

			// apply transformation from procrustes
			referencePoints = Matrix1.translate(referencePoints, -c.getX(), -c.getY());
			referencePoints = Matrix1.rotate(referencePoints, Math.acos(pro.getR().getEntry(0, 0)));
			referencePoints = Matrix1.translate(referencePoints, c.getX() + pro.getT().getEntry(0),
					c.getY() + pro.getT().getEntry(1));
			
			ColorProcessor procrustes = new ColorProcessor(Segmentation1.width, Segmentation1.height); 
			procrustes.invert();
			Visualize1.drawPoints(procrustes, referencePoints, Color.black);
			Visualize1.drawPoints(procrustes, targetPoints, Color.red);
			Visualize1.addToResults(procrustes, "Procrustes least squares fit");

			iterations++;

		} while (!match() && iterations < 5);

		finalReferencePoints = Matrix1.translate(finalReferencePoints,
				c_i.getCentroid().getX() - c_j.getCentroid().getX(),
				c_i.getCentroid().getY() - c_j.getCentroid().getY());

		if (match()) {
			if (Input1.showAssociations) {
				ColorProcessor finalAssoc = new ColorProcessor(Segmentation1.width, Segmentation1.height); 
				finalAssoc.invert();
				Visualize1.drawAssociations(finalAssoc, finalReferencePoints, finalTargetPoints);
				Visualize1.drawPoints(finalAssoc, finalReferencePoints, Color.black);
				Visualize1.drawPoints(finalAssoc, finalTargetPoints, Color.red);
				Visualize1.addToResults(finalAssoc, "Point correspondence");
			}
		}
	}

	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param originalPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private List<ClusterPoint1> getAssociation(List<ClusterPoint1> originalPositions,
			List<ClusterPoint1> targetPositions) {
		List<ClusterPoint1> assocPoints = new ArrayList<>();

		for (ClusterPoint1 original : originalPositions) {
			assocPoints.add(closestPoint(original, targetPositions));
		}
		return assocPoints;
	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param referencePoints
	 * @return
	 */

	private ClusterPoint1 closestPoint(ClusterPoint1 point, List<ClusterPoint1> referencePoints) {
		ClusterPoint1 closestPoint = null;
		double distance = Double.MAX_VALUE;

		for (ClusterPoint1 reference : referencePoints) {

			double distanceNew = reference.distance(point);

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = reference;
			}
		}
		
		tmp_error += Math.pow(distance,2);
		return closestPoint;
	}

	public List<ClusterPoint1> getAssociatedPoints() {
		return finalTargetPoints;
	}

	public List<ClusterPoint1> getTransformedPoints() {
		return finalReferencePoints;
	}

	public double getError() {
		return error;
	}
	
	public boolean match() {
		if (amountPoints < 5) {
			IJ.log("Amount is too low!");
			return true;
		}

		if (errorPerPoint < errorThreshold) {
			IJ.log("Match detected!");
			return true;
		}
		return false;
	}

	private ClusterPoint1 calculateCentroid(List<ClusterPoint1> points) {
		double avgX = 0.0;
		double avgY = 0.0;

		for (ClusterPoint1 point : points) {
			avgX += point.getX();
			avgY += point.getY();
		}

		return new ClusterPoint1(avgX / points.size(), avgY / points.size());
	}

	private List<double[]> getPoints(List<ClusterPoint1> input) {
		List<double[]> result = new ArrayList<>();

		for (ClusterPoint1 point : input) {
			result.add(point.getCoordinates());
		}

		return result;
	}
}
