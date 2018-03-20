package prototyping;

import java.util.List;

import NonRigid2D._Matrix;
import ij.IJ;
import ij.process.ImageProcessor;

public class PointsOrientation {

	private List<double[]> points;
	private double[] centroid;
	private double orientation;
	private double[] minBB = new double[2];

	private int length = 100;

	public PointsOrientation(List<double[]> points) {

		this.points = points;
		centroid = PointCollection.calculateCentroid(points);

		calculateOrientation();

	}

	private void calculateOrientation() {

		orientation = Math.atan2((2 * centralMoment(1, 1)), centralMoment(2, 0) - centralMoment(0, 2)) / 2.0;
	}

	private double centralMoment(int p, int q) {

		double moment = 0.0;

		for (double[] point : points) {
			moment += Math.pow((point[0] - centroid[0]), p) * Math.pow((point[1] - centroid[1]), q);
		}

		return moment;

	}

	public List<double[]> allignAxis() {
		
		points = _Matrix.translate(points, -centroid[0], -centroid[1]);
		points = _Matrix.rotate(points, -orientation);
		points = _Matrix.translate(points, centroid[0], centroid[1]);
		
		orientation = 0;
		
		return points;
	}

	public void drawPrincipalAxis(ImageProcessor cp) {

		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];
		
		int x_test = (int) centroid[0] - 30;
		
		int x2 = (int) (x1 + length/2 * Math.cos(orientation));
		int y2 = (int) (y1 + length/2 * Math.sin(orientation));	
		int x2_ = (int) (x1 - (length/2) * Math.cos(orientation));
		int y2_ = (int) (y1 - (length/2) * Math.sin(orientation));
		
		int y = (int) (Math.sin(orientation) * (x_test - x1) + y1);


		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
	}
	
	public void drawSecondaryAxis(ImageProcessor cp) {
		
		int x1 = (int) centroid[0];
		int y1 = (int) centroid[1];
		
		int x2 = (int) (x1 + (length/4) * Math.cos(orientation+1.5707963268));
		int y2 = (int) (y1 + (length/4) * Math.sin(orientation+1.5707963268));
		int x2_ = (int) (x1 - (length/4) * Math.cos(orientation+1.5707963268));
		int y2_ = (int) (y1 - (length/4) * Math.sin(orientation+1.5707963268));
		
		cp.drawLine(x1, y1, x2, y2);
		cp.drawLine(x1, y1, x2_, y2_);
		
	}

	public double getOrientation() {
		return orientation;
	}

	public double[] getCentroid() {
		return centroid;
	}
	
	public double[] getMinBB(){
		
		int minX = 1000;
		int maxY = 0;
		
		for(int i = 0; i < points.size(); i++){
			
			if(points.get(i)[0] < minX){
				minX = (int) points.get(i)[0];
			}
			
			if(points.get(i)[1] > maxY){
				maxY = (int) points.get(i)[1];
			}
			
		}
		
		minBB[0] = minX;
		minBB[1] = maxY;
	
		return minBB;
		
	}
	
	public int getPrincipalRadius(){
		
		int minX = Integer.MAX_VALUE;
		int maxX = 0;
		

		for(int i = 0; i < points.size(); i++){
			
			if(points.get(i)[0] < minX){
				minX = (int) points.get(i)[0];
			}
			
			if(points.get(i)[0] > maxX){
				maxX = (int) points.get(i)[0];
			}
			
		}
		
		return (maxX - minX)/2;
		
	}
	
public int getSecondaryRadius(){
		
		int minY = Integer.MAX_VALUE;
		int maxY = 0;
		

		for(int i = 0; i < points.size(); i++){
			
			if(points.get(i)[1] < minY){
				minY = (int) points.get(i)[1];
			}
			
			if(points.get(i)[1] > maxY){
				maxY = (int) points.get(i)[1];
			}
			
		}
		
		return (maxY - minY)/2;
		
	}

}
