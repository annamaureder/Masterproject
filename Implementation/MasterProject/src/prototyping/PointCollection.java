package prototyping;

import java.util.List;
import java.util.ArrayList;
import ij.IJ;
import ij.process.ImageProcessor;

/**
 * This class collects all points from an ImageProcessor and returns them in a point array
 * 
 * @author WB
 *
 */
public class PointCollection {
	
	public static List<double[]> getPoints(ImageProcessor ip) {

		int w = ip.getWidth();
		int h = ip.getHeight();
		
		List<double[]> pntlist = new ArrayList<double[]>();
		
		for (int v = 0; v < h; v++) {
			for (int u = 0; u < w; u++) {
				int p = ip.getPixel(u, v);
				
				if (p < -1) {
					pntlist.add(new double[]{u, v});
				}
			}
		}

		return pntlist;

	}
	
	public static double[] calculateCentroid(List<double[]> points){
		
		double avgX = 0.0;
		double avgY = 0.0;

		for (int i = 0; i < points.size(); i++) {
			avgX += points.get(i)[0];
			avgY += points.get(i)[1];
		}
		return new double[]{avgX/points.size(), avgY/points.size()};
	}
	
	
	
	
}
