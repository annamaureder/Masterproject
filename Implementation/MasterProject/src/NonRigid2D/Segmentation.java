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
 * unknown rigid parts For that a "divide and conquer" approach is implemented
 * to recursively and iteratively match clusters in both point clouds, until the
 * matched error works.
 * 
 * @version 2013/08/22
 */
public class Segmentation implements PlugInFilter {

	// variable declaration

	private ImagePlus im;
	public static int width;
	public static int height;
	private ClusterTree tree;
	List<Cluster[]> segmentedClusters;
	List<Cluster[]> rigidParts;
	
	public static ColorProcessor finalAssoc; 
	public static ColorProcessor assoc; 

	// private ClusterTree pose2;

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		return DOES_ALL + NO_CHANGES;
	}

	public void run(ImageProcessor ip) {

		ImageStack stack = im.getStack();
		width = im.getWidth();
		height = im.getHeight();
		finalAssoc = new ColorProcessor(width, height);
		finalAssoc.invert();
		
		assoc = new ColorProcessor(width, height);
		assoc.invert();

		ImageProcessor p1 = stack.getProcessor(1);
		ImageProcessor p2 = stack.getProcessor(2);

		Cluster S_0 = new Cluster(p1, true);
		Cluster D_0 = new Cluster(p2, true);

		Cluster cS_0 = new Cluster(p1, true);
		Cluster cD_0 = new Cluster(p2, true);

		tree = new ClusterTree(cS_0, cD_0);

		// recursive algorithm to subdivide the input clusters
		if (tree.checkClusters()) {
			segmentedClusters = tree.getSegmentedParts();
			rigidParts = tree.getRigidParts();
		}
		
		//ToDo:
		// - handle points at joints (weights, ignore them, fit ellipsoids)
		// - draw principal axis of all rigid parts P and draw the intersection as joints

		IJ.log("Number of clusters: " + segmentedClusters.size());
		IJ.log("Number of rigid parts: " + rigidParts.size());

		Visualize.colorSegments(segmentedClusters, "Cluster Segmentation");
		Visualize.colorSegments(rigidParts, "Rigid parts");
	

		ColorProcessor inputPoints = new ColorProcessor(width, height);
		inputPoints.invert();

		Visualize.drawPoints(inputPoints, S_0.getPoints(), Color.black);
		Visualize.drawPoints(inputPoints, D_0.getPoints(), Color.black);

		Visualize.showImage(inputPoints, "input points");

		ColorProcessor inputPoints_noNoise = new ColorProcessor(width, height);
		inputPoints_noNoise.invert();

		Visualize.drawPoints(inputPoints_noNoise, cS_0.getPoints(), Color.black);
		Visualize.drawPoints(inputPoints_noNoise, cD_0.getPoints(), Color.black);

		Visualize.showImage(inputPoints_noNoise, "input points");
		
		Visualize.showImage(finalAssoc, "Final Associations");
		Visualize.showImage(assoc, "Associations");

	}
	

}
