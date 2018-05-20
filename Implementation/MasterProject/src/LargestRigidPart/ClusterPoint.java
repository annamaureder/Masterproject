package LargestRigidPart;

import java.awt.geom.Point2D;
import java.util.List;

import ij.IJ;

public class ClusterPoint extends Point2D implements Comparable<ClusterPoint>{
	
	private double[] coordinates = new double[2];
	private double[] normal = new double[2];
	
	List<ClusterPoint> neighbors;
	private ClusterPoint closestPoint;
	public ClusterPoint getClosestPoint() {
		return closestPoint;
	}

	public void setClosestPoint(ClusterPoint closestPoint) {
		this.closestPoint = closestPoint;
	}

	private int[] FPFH;
	
	public ClusterPoint(double x, double y){
		coordinates[0] = x;
		coordinates[1] = y;
	}
	
	public ClusterPoint(double[] points){
		coordinates = points;
	}
	
	public double[] subtract(ClusterPoint p){
		double[] result = new double[p.coordinates.length];
		for (int i = 0; i < p.coordinates.length; i++){
			result[i] = this.coordinates[i] - p.coordinates[i];
		}
		return result;
	}
	
	public double[] getCoordinates() {
		return coordinates;
	}

	public void setCoordinates(double[] coordinates) {
		this.coordinates = coordinates;
	}

	public double[] getNormal() {
		return normal;
	}
	
	public void setNormal(double[] normal) {
		this.normal = normal;
	}

	public int[] getFPFH() {
		return FPFH;
	}
	
	public void setFPFH(int[] featureHistogram) {
		FPFH = featureHistogram;
	}
	
	public double getX(){
		return coordinates[0];
	}
	
	public double getY(){
		return coordinates[1];
	}
	
	public List<ClusterPoint> getNeighborhood() {
		return neighbors;
	}

	public void setNeighbors(List<ClusterPoint> neighbors) {
		this.neighbors = neighbors;
	}

	public void setLocation(double x, double y) {
		coordinates[0] = x;
		coordinates[1] = y;
		
	}
	
	public double dot(ClusterPoint p2){
		double scalar = 0.0;
        for (int i = 0; i < p2.getNormal().length; i++) {
            scalar += p2.getNormal()[i] * this.getNormal()[i];    
        }
        return scalar;
	}

	@Override
	public int compareTo(ClusterPoint o) {
		// TODO Auto-generated method stub
		return 0;
	}
}
