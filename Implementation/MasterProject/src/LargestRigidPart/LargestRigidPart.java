package LargestRigidPart;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;

import ij.IJ;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import procrustes.ProcrustesFit;
import prototyping.ICP;

/**
 * This plugin takes as input two point clouds c1 and c2 and returns the largest
 * rigid part by applying the ICP and a RANSAC approach with Procrustes
 * fitting.Ï
 * 
 * @version 2013/08/22
 */
public class LargestRigidPart {

	// variable declaration

	private Cluster c_i;
	private Cluster c_j;
	
	private Cluster[] lrp;

	public LargestRigidPart(Cluster c_i, Cluster c_j, List<List<double[]>> correspondances) {

		this.c_i = new Cluster(c_i);
		this.c_j = new Cluster(c_j);

		run();
	}

	private void run() {
		
		//ToDo: apply RANSAC on the correspondances and select the largest cluster from region growing from c_i and c_j
		
		
		
	}

	public Cluster[] getClusters() {
		return lrp;
	}

}
