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

/**
 * This plugin segments a non-rigid object given in two different poses into its unknown rigid parts
 * For that a "divide and conquer" approach is implemented to recursively and iteratively match clusters in both point clouds,
 * until the matched error works.
 * @version 2013/08/22
 */
public class Points2D_EM_unknownParts implements PlugInFilter {

	// variable declaration
	
	private ImagePlus im;



	public int setup(String arg, ImagePlus im) {
		this.im = im;
		return DOES_ALL + NO_CHANGES;
	}

	public void run(ImageProcessor ip) {

		/* ToDo:
		 * 
		 * i = 0
		 * 
		 * DIVIDE both point clouds into clusters
		 * if left(error > maxErrorPerPoint) --> DIVIDE again of clusterLeft (cluster is more than one part)
		 * else shift until maxErrorPerPoint doesn't get bigger in direction of maxError --> save as part_i++
		 * 
		 * if right(error > maxErrorPerPoint) --> DIVIDE again of clusterRight (cluster is more than one part)
		 * else shit until max ErrorPerPoint doesn't get bigger in direction of maxError --> save as part_i++
		 * 
		 * Repeat until all points matched
		 * 
		 */
		



		
	}

	
	
	
	void showImage(ImageProcessor ip, String title) {
		(new ImagePlus(title, ip)).show();
	}
}
