package LargestRigidPart;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.List;

import ij.IJ;
import ij.ImagePlus;
import ij.ImageStack;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import ij.process.ImageProcessor;

public class Visualize {
	
	private static ImageStack resultImages = new ImageStack(Main.width, Main.height);

	private static Color[] colors = new Color[] { Color.black, Color.red, Color.blue, Color.green, Color.magenta };

	public static void drawPoints(ColorProcessor cp, List<ClusterPoint> points, Color color) {
		cp.setColor(color);
		for (int i = 0; i < points.size(); i++) {
			cp.fillOval((int) points.get(i).getX(), (int) points.get(i).getY(), 3, 3);
		}
	}

	public static void drawDot(ColorProcessor cp, ClusterPoint point, Color color, int size) {
		cp.setColor(color);
		cp.fillOval((int) point.getX(), (int) point.getY(), size, size);
	}

	public static void colorClusters(List<Cluster[]> rigidParts, String title) {
		ColorProcessor segmentation = new ColorProcessor(Main.width, Main.height);
		ColorProcessor segmentation2 = new ColorProcessor(Main.width, Main.height);
		segmentation.invert();
		segmentation2.invert();

		for (int i = 0; i < rigidParts.size(); i++) {

			Cluster[] clusters = rigidParts.get(i);
			Cluster cluster1 = clusters[0];
			Cluster cluster2 = clusters[1];

			IJ.log("rigid part reference: " + cluster1.getPoints().size());
			IJ.log("rigid part reference: " + cluster1.getPoints().size());

			drawPoints(segmentation, cluster1.getPoints(), colors[i % colors.length]);
			drawPoints(segmentation2, cluster2.getPoints(), colors[i % colors.length]);

			if (cluster1.getJoint() != null && cluster2.getJoint() != null) {
				drawDot(segmentation, cluster1.getJoint(), colors[i % colors.length], 10);
				drawDot(segmentation2, cluster2.getJoint(), colors[i % colors.length], 10);
			}
		}
		if (Input.drawAxis) {
			drawSkeleton(segmentation, segmentation2, rigidParts);
		}
		addToResults(segmentation, title);
		addToResults(segmentation2, title);
	}

	public static void drawAssociations(ColorProcessor ip, List<ClusterPoint> points1, List<ClusterPoint> association) {
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

	public static void drawLine(ColorProcessor ip, ClusterPoint point1, ClusterPoint point2, Color color) {
		ip.setColor(color);

		Path2D line = new Path2D.Double();
		line.moveTo(point1.getX(), point1.getY());
		line.lineTo(point2.getX(), point2.getY());
		ShapeRoi roi1 = new ShapeRoi(line);
		roi1.setStrokeWidth(0.2f);
		roi1.setStrokeColor(color);
		ip.draw(roi1);
	}

	public static void drawSkeleton(ColorProcessor cp, ColorProcessor cp2, List<Cluster[]> segments) {
		for (int i = 0; i < segments.size(); i++) {
			segments.get(i)[0].drawPrincipalAxis(cp);
			segments.get(i)[1].drawPrincipalAxis(cp2);
		}
	}

	public static void addToResults(ImageProcessor ip, String title) {
		if(ip.getHeight() == resultImages.getHeight() && ip.getWidth() == resultImages.getWidth()){
			resultImages.addSlice(title, ip);
		} else{
			new ImagePlus(title, ip).show();
		}
	}
	
	public static void showResults(){
		new ImagePlus("Result Stack", resultImages).show();
	}
}
