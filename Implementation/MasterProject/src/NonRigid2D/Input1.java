package NonRigid2D;

import ij.gui.GenericDialog;

public class Input1 {
	
	public static boolean removeOutliers;
	public static boolean showInputCloud;
	public static boolean regionGrowing;
	public static boolean drawAxis;
	public static boolean showClusters;
	public static boolean showRigidParts;
	public static boolean showAssociations;
	public static double errorThreshold;
	public static double distanceThreshold;
	
	public static boolean getUserInput() {
		GenericDialog gd = new GenericDialog("Create Circle Test Image");
		gd.addCheckbox("Remove outliers", true);
		gd.addCheckbox("Show input point cloud", false);
		gd.addCheckbox("Region growing", false);
		gd.addCheckbox("Draw rigid part axis", false);
		gd.addCheckbox("Show segmented clusters", true);
		gd.addCheckbox("Show rigid parts", true);
		gd.addCheckbox("Show point correspondance", false);
		gd.addNumericField("Error threshold per point", 5.0, 2);
		gd.addNumericField("Region growing threshold", 10.0, 2);
		gd.showDialog();
		if (gd.wasCanceled()) {
			return false;
		}
		removeOutliers = gd.getNextBoolean();
		showInputCloud = gd.getNextBoolean();
		regionGrowing = gd.getNextBoolean();
		drawAxis = gd.getNextBoolean();
		showClusters = gd.getNextBoolean();
		showRigidParts = gd.getNextBoolean();
		showAssociations = gd.getNextBoolean();
		errorThreshold = gd.getNextNumber();
		distanceThreshold = gd.getNextNumber();
		return true;
	}

}
