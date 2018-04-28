package LargestRigidPart;

import ij.gui.GenericDialog;

public class Input {
	
	public static boolean removeOutliers;
	public static boolean showInputCloud;
	public static boolean drawAxis;
	public static boolean showRigidParts;
	public static boolean showAssociations;
	public static double errorThreshold;
	public static double distanceThreshold;
	
	public static boolean getUserInput() {
		GenericDialog gd = new GenericDialog("Create Circle Test Image");
		gd.addCheckbox("Remove outliers", true);
		gd.addCheckbox("Show input point cloud", false);
		gd.addCheckbox("Draw rigid part axis", true);
		gd.addCheckbox("Show rigid parts", true);
		gd.addCheckbox("Show point correspondance", false);
		gd.addNumericField("Error threshold per point", 6.0, 2);
		gd.addNumericField("Error threshold per point", 10.0, 2);
		gd.showDialog();
		if (gd.wasCanceled()) {
			return false;
		}
		removeOutliers = gd.getNextBoolean();
		showInputCloud = gd.getNextBoolean();
		drawAxis = gd.getNextBoolean();
		showRigidParts = gd.getNextBoolean();
		showAssociations = gd.getNextBoolean();
		errorThreshold = gd.getNextNumber();
		distanceThreshold = gd.getNextNumber();
		return true;
	}

}
