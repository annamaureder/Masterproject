package NonRigid2D;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.List;

import ij.ImagePlus;
import ij.gui.ShapeRoi;
import ij.process.ColorProcessor;
import ij.process.ImageProcessor;

public class Visualize1 {
	
	private static Color[] colors = new Color[] { Color.black, Color.red, Color.blue, Color.green, Color.magenta
			};
		
	public static void drawPoints(ColorProcessor cp, List<double[]> points, Color color) {
		cp.setColor(color);
		for (int i = 0; i < points.size(); i++) {
			cp.drawDot((int) points.get(i)[0], (int) points.get(i)[1]);
		}
	}
	
	public static void colorClusters(List<Cluster1[]> segments, String title) {
		ColorProcessor segmentation = new ColorProcessor(Segmentation1.width, Segmentation1.height);
		ColorProcessor segmentation2 = new ColorProcessor(Segmentation1.width, Segmentation1.height);
		segmentation.invert();
		segmentation2.invert();

		for (int i = 0; i < segments.size(); i++) {

			Cluster1[] clusters = segments.get(i);

			Cluster1 cluster1 = clusters[0];
			Cluster1 cluster2 = clusters[1];

			drawPoints(segmentation, cluster1.getPoints(), colors[i % colors.length]);
			drawPoints(segmentation2, cluster2.getPoints(), colors[i % colors.length]);
		}
		if(Input1.drawAxis){
			drawSkeleton(segmentation, segmentation2, segments);
		}
		showImage(segmentation, title);
		showImage(segmentation2, title);
	}
	
	public static void drawAssociations(ColorProcessor ip, List<double[]> points1, List<double[]> association) {
		ip.setColor(Color.green);
		for (int i = 0; i < points1.size(); i++) {

			if (association.get(i) != null) {
				Path2D line = new Path2D.Double();
				line.moveTo(points1.get(i)[0], points1.get(i)[1]);
				line.lineTo(association.get(i)[0], association.get(i)[1]);
				ShapeRoi roi1 = new ShapeRoi(line);
				roi1.setStrokeWidth(0.2f);
				roi1.setStrokeColor(Color.green);
				ip.draw(roi1);
			}
		}
	}
	
	public static void drawSkeleton(ColorProcessor cp, ColorProcessor cp2, List<Cluster1[]> segments){
		for(int i = 0; i < segments.size(); i++){
			segments.get(i)[0].drawPrincipalAxis(cp);
			segments.get(i)[1].drawPrincipalAxis(cp2);
		}
	}
	
	public static void showImage(ImageProcessor ip, String title) {
		(new ImagePlus(title, ip)).show();
	}
	

}
