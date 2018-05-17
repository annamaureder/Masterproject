package LargestRigidPart;

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

import apple.laf.JRSUIUtils.Tree;

/**
 * This plugin segments a non-rigid object given in two different poses into its
 * unknown rigid parts. For that a "divide and conquer" approach is implemented
 * to recursively subdivide both point clouds, until the
 * matching error between all corresponding sub clusters of cloudA and cloudB is below a fixed threshold.
 * 
 * @version 2013/08/22
 */
public class Segmentation implements PlugInFilter {

	// static variables
	public static int width;
	public static int height;
	public static ColorProcessor finalAssoc;

	// member variables

	private ImagePlus im;
	private List<Cluster[]> rigidParts;
	
	private Cluster c1;
	private Cluster c2;

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		width = im.getWidth();
		height = im.getHeight();
		return DOES_ALL + STACK_REQUIRED;
	}

	public void run(ImageProcessor ip) {
		
		if (!Input.getUserInput()) {
			return;
		}
		
		ImageStack stack = im.getStack();
		ImageProcessor p1 = stack.getProcessor(1);
		ImageProcessor p2 = stack.getProcessor(2);

		c1 = new Cluster(p1);
		c2 = new Cluster(p2);
		
		IJ.log("Cluster1: " + c1.getPoints().size() + " points");
		IJ.log("Cluster2: " + c2.getPoints().size() + " points");
		
		c1.calculateFeatures();
		c2.calculateFeatures();
		
		showNormals();
		
		//PartDetection detect = new PartDetection(c1, c2);
		//rigidParts = detect.getRigidParts();
		

		//showResults();
	}

	private void showNormals() {
		ColorProcessor cp = new ColorProcessor(width, height);
		cp.invert();
		
		for(ClusterPoint point : c1.getPoints()){
			if(point.getNormal()==null){
				IJ.log("Normal is null");
				return;
			}
		}
		
		Visualize.showImage(cp, "Normals C1");
		
		ColorProcessor cp2 = new ColorProcessor(width, height);
		cp2.invert();
		
		for(ClusterPoint point : c2.getPoints()){
			if(point.getNormal()==null){
				IJ.log("Normal is null");
				return;
			}
			
			IJ.log("normal: " + point.getNormal()[0] + "/" + point.getNormal()[1]);
			Visualize.drawDot(cp2, point, Color.black, 4);
			ClusterPoint target = new ClusterPoint(point.getX() + point.getNormal()[0] * 10, point.getY() + point.getNormal()[1] * 10);
			Visualize.drawLine(cp2, point, target, Color.green);
			Visualize.drawDot(cp2, target, Color.red,2);
		}
		
		Visualize.showImage(cp2, "Normals C2");
	}
	
	/**
	 * method to visualize all results
	 */
	public void showResults(){
		
		if(Input.showRigidParts){
			Visualize.colorClusters(rigidParts, "RigidParts");
		}
		
		if(Input.showInputCloud){
			ColorProcessor inputPoints_1 = new ColorProcessor(width, height);
			inputPoints_1.invert();
			
			ColorProcessor inputPoints_2 = new ColorProcessor(width, height);
			inputPoints_2.invert();

			Visualize.drawPoints(inputPoints_1, c1.getPoints(), Color.black);
			Visualize.drawPoints(inputPoints_2, c2.getPoints(), Color.black);

			Visualize.showImage(inputPoints_1, "Input points 1");
			Visualize.showImage(inputPoints_2, "Input points 2");
		}
	}
}
