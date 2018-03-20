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
public class _Segmentation implements PlugInFilter {

	// static variables
	public static int width;
	public static int height;
	public static ColorProcessor finalAssoc; 
	public static ColorProcessor assoc; 

	//member variables
	
	private ImagePlus im;
	private _ClusterTree tree;
	private List<_Cluster[]> segmentedClusters;
	private List<_Cluster[]> rigidParts;

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		width = im.getWidth();
		height = im.getHeight();
		return DOES_ALL + NO_CHANGES;		
	}

	public void run(ImageProcessor ip) {
		
		ImageStack stack = im.getStack();
		ImageProcessor p1 = stack.getProcessor(1);
		ImageProcessor p2 = stack.getProcessor(2);

		_Cluster c0_0 = new _Cluster(p1, true);
		_Cluster c1_0 = new _Cluster(p2, true);

		tree = new _ClusterTree(c0_0, c1_0);
		segmentedClusters = tree.subdivide(segmentedClusters);
		rigidParts = tree.getRigidParts(segmentedClusters);
		
		/*finalAssoc = new ColorProcessor(width, height);
		finalAssoc.invert();
		assoc = new ColorProcessor(width, height);
		assoc.invert();*/
		
		//ToDo:
		// - handle points at joints (weights, ignore them, fit ellipsoids)
		// - draw principal axis of all rigid parts P and draw the intersection as joints

		IJ.log("Number of clusters: " + segmentedClusters.size());
		IJ.log("Number of rigid parts: " + rigidParts.size());

		_Visualize.colorClusters(segmentedClusters, "Cluster Segmentation");
		_Visualize.colorClusters(rigidParts, "Rigid parts");
	

		ColorProcessor inputPoints = new ColorProcessor(width, height);
		inputPoints.invert();

		_Visualize.showImage(inputPoints, "input points");

		ColorProcessor inputPoints_noNoise = new ColorProcessor(width, height);
		inputPoints_noNoise.invert();

		_Visualize.drawPoints(inputPoints_noNoise, c0_0.getPoints(), Color.black);
		_Visualize.drawPoints(inputPoints_noNoise, c1_0.getPoints(), Color.black);

		_Visualize.showImage(inputPoints_noNoise, "input points");
		
		_Visualize.showImage(finalAssoc, "Final Associations");
		_Visualize.showImage(assoc, "Associations");

	}
	

}
