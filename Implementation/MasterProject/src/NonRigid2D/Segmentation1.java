package NonRigid2D;

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
public class Segmentation1 implements PlugInFilter {

	// static variables
	public static int width;
	public static int height;
	public static ColorProcessor finalAssoc;

	// member variables

	private ImagePlus im;
	private ClusterTree tree;
	private List<Cluster1[]> subclusters;
	private List<Cluster1[]> mergedParts;
	
	private Cluster1 c1;
	private Cluster1 c2;

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		width = im.getWidth();
		height = im.getHeight();
		return DOES_ALL + STACK_REQUIRED;
	}

	public void run(ImageProcessor ip) {
		
		if (!Input1.getUserInput()) {
			return;
		}
		
		if(Input1.showAssociations){
			finalAssoc = new ColorProcessor(width, height); 
			finalAssoc.invert();
		}

		ImageStack stack = im.getStack();
		ImageProcessor p1 = stack.getProcessor(1);
		ImageProcessor p2 = stack.getProcessor(2);

		c1 = new Cluster1(p1);
		c2 = new Cluster1(p2);

		tree = new ClusterTree(c1, c2);
		subclusters = tree.subdivide(tree.getRoot());
		IJ.log("Subdividing done!");
		mergedParts = tree.mergeClusters(subclusters);
		IJ.log("Merging done!");

		IJ.log("Number of clusters: " + subclusters.size());
		IJ.log("Number of rigid parts: " + mergedParts.size());
		
		showResults();
	}
	
	/**
	 * method to visualize all results
	 */
	public void showResults(){
		if(Input1.showClusters){
			Visualize1.colorClusters(subclusters, "Cluster Segmentation");
		}
		
		if(Input1.showRigidParts){
			Visualize1.colorClusters(mergedParts, "RigidParts");
		}
		
		if(Input1.showInputCloud){
			ColorProcessor inputPoints_1 = new ColorProcessor(width, height);
			inputPoints_1.invert();
			
			ColorProcessor inputPoints_2 = new ColorProcessor(width, height);
			inputPoints_2.invert();

			Visualize1.drawPoints(inputPoints_1, c1.getPoints(), Color.black);
			Visualize1.drawPoints(inputPoints_2, c2.getPoints(), Color.black);

			Visualize1.showImage(inputPoints_1, "Input points 1");
			Visualize1.showImage(inputPoints_2, "Input points 2");
		}
		if(Input1.showAssociations){
			Visualize1.showImage(finalAssoc, "Final Associations");
		}
	}
}
