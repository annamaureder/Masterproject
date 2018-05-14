package LargestRigidPart;

import java.util.ArrayList;
import java.util.List;

/*
 * ToDo
 * - Takes two clusters with cluster points as input
 * - calculates the normal for each cluster point (PCA)
 * - calculates the feature histogram(s) for each cluster point
 * - detects the most unique feature diagrams
 * - finds fitting feature diagrams between two points
 */

public class FPFH {
	
	private ClusterPoint point;
	private final int numberFeatures = 3;
	private final int numberIntervals = 2;
	private final int bins = (int) Math.pow(numberIntervals, numberFeatures);
	
	int [] SPFH;
	int [] weightedSPFH;
	int [] FPFH;
	
	public FPFH(ClusterPoint point){
		this.point = point;
	}
	
	public void detectFeatures(){
		SPFH = SPFH(point);
		
		for(ClusterPoint neighbor : point.getNeighborhood()){
			double weight = point.distance(neighbor);
			weightedSPFH = addHistograms(weightedSPFH, multiplyHistograms(SPFH(neighbor), (1.0/weight)));
		}
		
		FPFH = addHistograms(SPFH, multiplyHistograms(weightedSPFH, 1.0/point.getNeighborhood().size()));
		point.setFPFH(FPFH);
	}
	
	private int[] SPFH(ClusterPoint point){
		int[] histogram = new int[bins];
		double[] u;
		double[] v;
		double[] w;
		
		ClusterPoint p_i;
		ClusterPoint p_j;
		
		double[] n_i;
		double[] n_j;
		
		for(ClusterPoint neighbor : point.getNeighborhood()){
			//TODO: Check which ones angle is smaller! --> p_i, n_i DOT PRODUCT
			p_i = point;
			p_j = neighbor;
			n_i = point.getNormal();
			n_j = neighbor.getNormal();
			
			u = n_i;
			v = cross(p_j.subtract(p_i), u);
			w = cross(u,v);
			
			double feature1 = dot(v, n_j);
			double feature2 = dot(u, p_j.subtract(p_i))/p_j.distance(p_i); //TODO
			double feature3 = Math.atan(dot(w, n_j)); //TODO: projected on u??

			List<Double> features = new ArrayList<>();
			features.add(feature1);
			features.add(feature2);
			features.add(feature3);

			histogram[idx(features)]++;
		}

		return histogram;
	}
	
	private int[] addHistograms(int[] h1, int[] h2){
		int[] result = new int[h1.length];
		
		for(int i = 0; i < h1.length; i++){
			result[i] = h1[i] + h2[i];
		}
		return result;
	}
	
	private int[] multiplyHistograms(int[] h, double multiplier){
		int[] result = new int[h.length];
		
		for(int i = 0; i < h.length; i++){
			result[i] = (int) (h[i] * multiplier);
		}
		return result;
	}
	
	private double[] cross(double[] v1, double[] v2){
		if(v1.length == 2){
			double[] tmp = new double[3];
			tmp[0] = v1[0];
			tmp[1] = v1[1];
			tmp[2] = 0;
			v1 = tmp;
		}
		if(v2.length == 2){
			double[] tmp = new double[3];
			tmp[0] = v2[0];
			tmp[1] = v2[1];
			tmp[2] = 0;
			v2 = tmp;
		}
		
		double[] result = new double[3];
		result[0] = v1[1] * v2[2] - v1[2] * v2[1];
		result[1] = v1[2] * v2[0] - v1[0] * v2[2];
		result[2] = v1[0] * v2[1] - v1[1] * v2[0];
		
		return result;
	}
	
	private double dot(double[] v1, double[] v2){
		int scalar = 0;
        for (int i = 0; i < v1.length; i++) {
            scalar += v1[i] * v2[i];    
        }
        return scalar;
	}
	
	private int idx(List<Double> features){
		int index = 0;
		int interval = 0;
		
		for(int i = 0; i < features.size();i++){
			
			if(i == 0 || i == 1){
				interval = getInterval(features.get(i), -1, 1);
			}
			
			else{
				interval = getInterval(features.get(i), -Math.PI/2, Math.PI/2);
			}
			index += interval * Math.pow(2, i);
		}
		return index;
	}
	
	private int getInterval(double feature, double min, double max){
		int range = (int) (max - min)/numberIntervals;
		return (int)(feature - min) / range;
	}

}
