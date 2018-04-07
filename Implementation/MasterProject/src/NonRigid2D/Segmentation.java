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
public class Segmentation implements PlugInFilter {

	// static variables
	public static int width;
	public static int height;
	public static ColorProcessor finalAssoc;
	public static ColorProcessor assoc;

	// member variables

	private ImagePlus im;
	private ClusterTree tree;
	private List<Cluster[]> subclusters;
	private List<Cluster[]> mergedParts;

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

		Cluster c1 = new Cluster(p1, true);
		Cluster c2 = new Cluster(p2, true);

		tree = new ClusterTree(c1, c2);
		subclusters = tree.subdivide(tree.getRoot());
		mergedParts = tree.mergeClusters(subclusters);

		/*
		 * finalAssoc = new ColorProcessor(width, height); finalAssoc.invert();
		 * assoc = new ColorProcessor(width, height); assoc.invert();
		 */

		// ToDo:
		// - handle points at joints (weights, ignore them, fit ellipsoids)
		// - draw principal axis of all rigid parts P and draw the intersection
		// as joints

		IJ.log("Number of clusters: " + subclusters.size());
		IJ.log("Number of rigid parts: " + mergedParts.size());

		Visualize.colorClusters(subclusters, "Cluster Segmentation");
		Visualize.colorClusters(mergedParts, "Rigid parts");

		ColorProcessor inputPoints = new ColorProcessor(width, height);
		inputPoints.invert();

		Visualize.showImage(inputPoints, "input points");

		ColorProcessor inputPoints_noNoise = new ColorProcessor(width, height);
		inputPoints_noNoise.invert();

		Visualize.drawPoints(inputPoints_noNoise, c1.getPoints(), Color.black);
		Visualize.drawPoints(inputPoints_noNoise, c2.getPoints(), Color.black);

		Visualize.showImage(inputPoints_noNoise, "input points");

		Visualize.showImage(finalAssoc, "Final Associations");
		Visualize.showImage(assoc, "Associations");

	}

}
