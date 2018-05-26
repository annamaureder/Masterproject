package LargestRigidPart;

import ij.IJ;
import ij.gui.GenericDialog;

public class Input {
	
	public static boolean removeOutliers;
	public static boolean showInputCloud;
	public static boolean drawAxis;
	public static boolean showRigidParts;
	public static boolean showAssociations;
	public static boolean reciprocalMatching;
	public static double errorThreshold;
	public static double distanceThresholdRG;
	public static double distanceThresholdJoints;
	public static double distanceThresholdRANSAC;
	public static boolean logging;
	public static String distance;
	
	public static boolean getUserInput() {
		GenericDialog gd = new GenericDialog("Rigid part segmentation");
		gd.addCheckbox("Remove outliers", true);
		gd.addCheckbox("Show input point cloud", false);
		gd.addCheckbox("Draw rigid part axis", false);
		gd.addCheckbox("Show rigid parts", true);
		gd.addCheckbox("Show point correspondance", true);
		gd.addCheckbox("Reciprocal point matching", true);
		gd.addNumericField("Error threshold per point", 6.0, 2);
		gd.addNumericField("Distance threshold for region growing", 15.0, 2);
		gd.addNumericField("Distance threshold for Joint rotations", 5.0, 2);
		gd.addNumericField("Distance threshold for RANSAC", 3.0, 2);
		gd.addCheckbox("Activate logging", false);
		String[] distances = {"Euclidean", "ChiSquared", "Kullback-Leibler"};
        gd.addRadioButtonGroup("", distances, 1, 3, "Euclidean");
		gd.showDialog();
		if (gd.wasCanceled()) {
			return false;
		}
		removeOutliers = gd.getNextBoolean();
		showInputCloud = gd.getNextBoolean();
		drawAxis = gd.getNextBoolean();
		showRigidParts = gd.getNextBoolean();
		showAssociations = gd.getNextBoolean();
		reciprocalMatching = gd.getNextBoolean();
		errorThreshold = gd.getNextNumber();
		distanceThresholdRG = gd.getNextNumber();
		distanceThresholdJoints = gd.getNextNumber();
		distanceThresholdRANSAC = gd.getNextNumber();
		logging = gd.getNextBoolean();
		distance = gd.getNextRadioButton();
		
		IJ.log("Distance: " + distance);
				
		return true;
	}

}
