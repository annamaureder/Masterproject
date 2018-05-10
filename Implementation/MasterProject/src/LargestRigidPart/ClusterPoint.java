package LargestRigidPart;

public class ClusterPoint {
	
	private double[] coordinates = new double[2];
	private double[] normal = new double[2];
	private int[] FPFH;
	
	public ClusterPoint(double x, double y){
		coordinates[0] = x;
		coordinates[1] = y;
	}
	
	public ClusterPoint(double[] points){
		coordinates = points;
	}
	
	public double distance(ClusterPoint point2) {
		double x = Math.abs(this.coordinates[0] - point2.coordinates[0]);
		double y = Math.abs(this.coordinates[1] - point2.coordinates[1]);

		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
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

	public int[] getFPFH() {
		return FPFH;
	}
	
	public double getX(){
		return coordinates[0];
	}
	
	public double getY(){
		return coordinates[1];
	}
	
}
