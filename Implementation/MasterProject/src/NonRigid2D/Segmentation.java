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
 * This plugin segments a non-rigid object given in two different poses into its unknown rigid parts
 * For that a "divide and conquer" approach is implemented to recursively and iteratively match clusters in both point clouds,
 * until the matched error works.
 * @version 2013/08/22
 */
public class Segmentation implements PlugInFilter {

	// variable declaration
	
	private ImagePlus im;
	private ClusterTree tree;
	List<Cluster[]> segmentedClusters;
	List<Cluster[]> rigidParts;
	private Color[] colors = new Color[]{Color.black, Color.blue, Color.green, Color.red, Color.orange, Color.yellow};
	
	//private ClusterTree pose2;

	public int setup(String arg, ImagePlus im) {
		this.im = im;
		return DOES_ALL + NO_CHANGES;
	}

	public void run(ImageProcessor ip) {
		
		ImageStack stack = im.getStack();
		ImageProcessor p1 = stack.getProcessor(1);
		ImageProcessor p2 = stack.getProcessor(2);
		
		tree = new ClusterTree(new Cluster(p1), new Cluster(p2));
		
		/*
		 * ToDo:
		 * - add both original clusters into tree
		 * - divide and add nodes depending of error from ICP of current nodes (depth first
		 * - save cluster if no further segmentation is done for it (no childs)
		 * - check for the same amount of points!
		 * 
		 * 
		 */
		
		if(tree.checkClusters()){
			segmentedClusters = tree.getSegmentedParts();		
			rigidParts = tree.getRigidParts();
		}
						
			
			IJ.log("Number of clusters: " + segmentedClusters.size());
			IJ.log("Number of rigid parts: " + rigidParts.size());
			
			colorSegments(segmentedClusters, "Cluster Segmentation");
			colorSegments(rigidParts, "Rigid parts");
						
		}		


	void colorSegments(List <Cluster[]> segments, String title){
		
		ColorProcessor segmentation = new ColorProcessor(250, 250);
		segmentation.invert();

		
		for(int i = 0; i < segments.size(); i++){
			segmentation.setColor(colors[i%colors.length]);
			//segmentation.setColor(Color.red);

			Cluster[] clusters = segments.get(i);
			
			Cluster cluster1 = clusters[0];
			Cluster cluster2 = clusters[1];
			
			for(int x = 0; x < cluster1.getPoints().size(); x++){
				int[] point = new int[]{(int) cluster1.getPoints().get(x)[0], (int) cluster1.getPoints().get(x)[1]};
				segmentation.drawDot(point[0], point[1]);
			}
			
			for(int j = 0; j < cluster2.getPoints().size();j++){
				int[] point = new int[]{(int) cluster2.getPoints().get(j)[0], (int) cluster2.getPoints().get(j)[1]};
				segmentation.drawDot(point[0], point[1]);
			}
		}
		
		showImage(segmentation, title);
		
	}
	
	
	
	void showImage(ImageProcessor ip, String title) {
		(new ImagePlus(title, ip)).show();
	}

}
