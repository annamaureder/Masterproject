package prototyping;

import ij.IJ;
import ij.ImagePlus;
import ij.ImageStack;
import ij.gui.GenericDialog;
import ij.gui.Overlay;
import ij.gui.ShapeRoi;
import ij.plugin.filter.PlugInFilter;
import ij.process.ColorProcessor;
import ij.process.ImageProcessor;
import imagingbook.pub.corners.Corner;
import imagingbook.pub.corners.HarrisCornerDetector;
import procrustes.ProcrustesFit;

import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.apache.commons.math3.geometry.spherical.twod.Vertex;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import NonRigid2D._Matrix;


/**
 * This plugin implements the ICP for two 2D point clouds
 * 
 * @version 2013/08/22
 */
public class ICP {
	
	//variable declaration
	
	private List<double[]> points1;
	private List<double[]> points2;

	private double error = Double.MAX_VALUE;
	private double tmp_error = 0;

	private List<double[]> associatedPoints;
	private List<double[]> transformedPoints;
	
	private List<double[]> tmp_transformation;
	private List<double[]> tmp_association;
	
	double[] c1;
	double[] c2;
	
	public ICP(List<double[]> p1, List<double[]> p2){
		
		points1 = p1;
		points2 = p2;
		
		c1 = PointCollection.calculateCentroid(points1);
		c2 = PointCollection.calculateCentroid(points2);
		
		this.run();
	}
	
	private void run(){
	
	
		/*
		 * Step 1 - initial transformation estimate (move centroids to origin)
		 */
		
		
		List<double[]> points1_origin = _Matrix.translate(points1, -c1[0], -c1[1]);
		List<double[]> points2_origin = _Matrix.translate(points2, -c2[0], -c2[1]);
		
		/*
		 * Step 2 - associate Points
		 */

		tmp_association = new ArrayList<double[]>();
		tmp_transformation = new ArrayList<double[]>();
		
		int i = 0;
		
		while (i < 360) {
			
			//recalculate Transformation for "best fit"
				
			tmp_transformation = _Matrix.rotate(points1_origin, (i/180.0) * Math.PI);
			tmp_association = getAssociation(tmp_transformation, points2_origin);	
			
			if (tmp_error < error) {
				error = tmp_error;
				associatedPoints = tmp_association;
				transformedPoints = tmp_transformation;
			}
			
			i++;
		}
		
		associatedPoints = _Matrix.translate(associatedPoints, c2[0], c2[1]);
		transformedPoints = _Matrix.translate(transformedPoints, c2[0], c2[1]);
		
		ProcrustesFit pro = new ProcrustesFit();
		pro.fit(points1, associatedPoints);
		//error = pro.getError();
		
		//IJ.log("Rotation:" + pro.getR().getEntry(0, 0));
		//IJ.log("Transformation: " + pro.getT().getEntry(0) + "/" + pro.getT().getEntry(1));
		
		//transformedPoints = Matrix.rotate(points1_origin, Math.acos(pro.getR().getEntry(0, 0)));
		//transformedPoints = Matrix.translate(transformedPoints, c1[0] + pro.getT().getEntry(0), c1[1] + pro.getT().getEntry(1));
		
	}
	
	
	/**
	 * method to get associations between two point sets X and X'
	 * 
	 * @param resultPositions
	 * @param targetPositions
	 * @return List with points with the same sorting as resultPoitns
	 */
	private List<double[]> getAssociation(List<double[]> resultPositions, List<double[]> targetPositions) {

		tmp_error = 0;
		List<double[]> assocPoints = new ArrayList<double[]>();

		for (int i = 0; i < resultPositions.size(); i++) {
			assocPoints.add(closestPoint(resultPositions.get(i), targetPositions));
		}

		return assocPoints;

	}

	/**
	 * method to get the closest point for x' from a points set X'
	 * 
	 * @param point
	 * @param comparePoints
	 * @return
	 */

	private double[] closestPoint(double[] point, List<double[]> comparePoints) {

		double[] closestPoint = null;
		double distance = Double.MAX_VALUE;

		for (int i = 0; i < comparePoints.size(); i++) {

			double distanceNew = Math.sqrt(
					Math.pow(point[0] - comparePoints.get(i)[0], 2) + Math.pow(point[1] - comparePoints.get(i)[1], 2));

			if (distanceNew < distance) {
				distance = distanceNew;
				closestPoint = comparePoints.get(i);
			}
			
			tmp_error += distance;
		}

		return closestPoint;
	}
	
	public List<double[]> getAssociatedPoints(){
		return associatedPoints;
	}
	
	public List<double[]> getTransformedPoints(){
		return transformedPoints;
	}
	
	public double getError(){
		double errorPerPoint = error/((points1.size() + points2.size())/2.0);
		IJ.log("error per point: " + errorPerPoint);
		
		return error;
	}

}
