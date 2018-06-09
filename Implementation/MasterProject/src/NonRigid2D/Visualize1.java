package NonRigid2D;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.List;

import LargestRigidPart.Visualize;
import ij.IJ;
import ij.ImagePlus;
import ij.ImageStack;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import ij.process.ImageProcessor;

public class Visualize1 {

	private static ImageStack resultImages = new ImageStack(Segmentation1.width, Segmentation1.height);

	private static Color[] colors = new Color[] { Color.black, Color.red, Color.blue, Color.green, Color.magenta };

	public static void drawPoints(ColorProcessor cp, List<ClusterPoint1> points, Color color) {
		cp.setColor(color);
		for (int i = 0; i < points.size(); i++) {
			cp.fillOval((int) points.get(i).getX(), (int) points.get(i).getY(), 3, 3);
		}
	}

	public static void drawDot(ColorProcessor cp, ClusterPoint1 point, Color color, int size) {
		cp.setColor(color);
		cp.fillOval((int) point.getX(), (int) point.getY(), size, size);
	}

	public static void colorClusters(List<Cluster1[]> rigidParts, String title) {
		ColorProcessor segmentation = new ColorProcessor(Segmentation1.width, Segmentation1.height);
		ColorProcessor segmentation2 = new ColorProcessor(Segmentation1.width, Segmentation1.height);
		segmentation.invert();
		segmentation2.invert();

		int i = 0;
		for (Cluster1[] clusters : rigidParts) {
			Cluster1 cluster1 = clusters[0];
			Cluster1 cluster2 = clusters[1];

			if (cluster1.getJoint() != null) {
				IJ.log("Joint detected: " + cluster1.getJoint().getX() + "/" + cluster1.getJoint().getY());
				drawDot(segmentation, cluster1.getJoint(), colors[i % colors.length], 10);
				drawDot(segmentation2, cluster2.getJoint(), colors[i % colors.length], 10);
			}

			drawPoints(segmentation, cluster1.getPoints(), colors[i % colors.length]);
			drawPoints(segmentation2, cluster2.getPoints(), colors[i++ % colors.length]);
		}
		if (Input1.drawAxis) {
			drawSkeleton(segmentation, segmentation2, rigidParts);
		}
		addToResults(segmentation, title);
		addToResults(segmentation2, title);
	}

	public static void drawAssociations(ColorProcessor ip, List<ClusterPoint1> points1,
			List<ClusterPoint1> association) {
		ip.setColor(Color.green);
		for (int i = 0; i < points1.size(); i++) {

			if (association.get(i) != null) {
				Path2D line = new Path2D.Double();
				line.moveTo(points1.get(i).getX(), points1.get(i).getY());
				line.lineTo(association.get(i).getX(), association.get(i).getY());
				ShapeRoi roi1 = new ShapeRoi(line);
				roi1.setStrokeWidth(0.2f);
				roi1.setStrokeColor(Color.green);
				ip.draw(roi1);
			}
		}
	}

	public static void drawLine(ColorProcessor ip, ClusterPoint1 point1, ClusterPoint1 point2, Color color) {
		ip.setColor(color);

		Path2D line = new Path2D.Double();
		line.moveTo(point1.getX(), point1.getY());
		line.lineTo(point2.getX(), point2.getY());
		ShapeRoi roi1 = new ShapeRoi(line);
		roi1.setStrokeWidth(0.2f);
		roi1.setStrokeColor(color);
		ip.draw(roi1);
	}

	public static void drawSkeleton(ColorProcessor cp, ColorProcessor cp2, List<Cluster1[]> rigidParts) {
		for (int i = 0; i < rigidParts.size(); i++) {
			rigidParts.get(i)[0].drawPrincipalAxis(cp);
			rigidParts.get(i)[1].drawPrincipalAxis(cp2);
		}
	}

	public static void addToResults(ImageProcessor ip, String title) {
		if (ip.getHeight() == resultImages.getHeight() && ip.getWidth() == resultImages.getWidth()) {
			resultImages.addSlice(title, ip);
		} else {
			new ImagePlus(title, ip).show();
		}
	}

	public static void showResults() {
		new ImagePlus("Result Stack", resultImages).show();
	}
}
