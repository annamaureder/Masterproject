package LargestRigidPart;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;

import ij.IJ;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;

/**
 * This plugin takes as input two point clouds c1 and c2 and returns the largest
 * rigid part by applying the ICP and a RANSAC approach with Procrustes
 * fitting.Ï
 * 
 * @version 2013/08/22
 */
public class PartDetection {

	// variable declaration

	private Cluster c_i;
	private Cluster c_j;
	
	private List<double[]> denseCorrespondances_c1;
	private List<double[]> denseCorrespondances_c2;
	
	private List<Cluster[]> lrp = new ArrayList();

	public PartDetection(Cluster c_i, Cluster c_j) {

		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		run();
	}

	private void run() {
		
		ClosestPoint cp = new ClosestPoint(c_i, c_j);
		
		denseCorrespondances_c1 = cp.getReferencePoints();
		denseCorrespondances_c2 = cp.getTargetPoints();
		
		IJ.log("Finished!");
		
		
		//Create recursive loop to detect all lrps of C1 and C2.
		
		
		
		
		
		
		
		List<List<double[]>> denseCorrespondances = new ArrayList<>();
		denseCorrespondances.add(denseCorrespondances_c1);
		denseCorrespondances.add(denseCorrespondances_c2);
		
		lrp.add(new LargestRigidPart(c_i, c_j, denseCorrespondances).getClusters());
	}
	
	public List<Cluster[]> getRigidParts(){
		return lrp;
	}
}
