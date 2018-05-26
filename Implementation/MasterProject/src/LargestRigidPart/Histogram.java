package LargestRigidPart;

import java.awt.Color;
import java.util.List;

import ij.IJ;
import ij.gui.Plot;
import ij.gui.PlotWindow;
import java.awt.*;

public class Histogram{
	
	private int[] histogram;
	private int bins;
	
	public Histogram(int bins){
		this.bins = bins;
		histogram = new int[bins];
	}
	public void showHistogram(){
		BarChart.run(new String[0]);
	}
	
	public void showTestData(){
        float[] x = new float[bins];
        float[] y = new float[bins];
        for(int i = 0; i < bins; i++){
        	x[i] = i;
        	y[i] = i;
        	IJ.log("Value: " + histogram[i]);
        }

        PlotWindow.noGridLines = false; // draw grid lines
        Plot plot = new Plot("Feature histograms","bins","",x,y);
        plot.setLimits(0, bins-1, 0, 10);
        plot.setLineWidth(2);

        // add label
        plot.setColor(Color.black);
        plot.changeFont(new Font("Helvetica", Font.PLAIN, 24));

        plot.changeFont(new Font("Helvetica", Font.PLAIN, 16));
        plot.setColor(Color.blue);
        plot.show();
	}
	
	public static Histogram meanHistogram(List<Histogram> histograms){
		Histogram mean = new Histogram(histograms.get(0).getBins());
		
		for(Histogram histogram : histograms){
			for(int i = 0; i < histogram.getHistogram().length; i++){
				mean.getHistogram()[i] += histogram.getHistogram()[i];
			}
		}
		
		for(int i = 0; i < mean.getHistogram().length; i++){
			mean.getHistogram()[i] /= histograms.size();
		}
		return mean;
	}
	
	public double squaredDistance(Histogram h2){
		double distance = 0.0;
		for(int i = 0; i < h2.getHistogram().length; i++){
			distance+= Math.pow(this.getHistogram()[i] - h2.getHistogram()[i], 2);
		}
		return distance;
	}
	
	public double chiSquare(Histogram h2){
		double distance = 0.0;
		for(int i = 0; i < h2.getHistogram().length; i++){
			double numerator = Math.pow(this.getHistogram()[i] - h2.getHistogram()[i], 2) == 0 ? 1 : Math.pow(this.getHistogram()[i] - h2.getHistogram()[i], 2);
			double divider = (this.getHistogram()[i] + h2.getHistogram()[i]) == 0 ? 1 : (this.getHistogram()[i] + h2.getHistogram()[i]);
			distance+= numerator / divider;
		}
		return distance;
	}
	
	public double kullback(Histogram h2){
		double distance = 0.0;
		
		for(int i = 0; i < h2.getHistogram().length; i++){
			double divider = h2.getHistogram()[i] == 0 ? 1 : h2.getHistogram()[i];
			double numerator = this.getHistogram()[i] == 0 ? 1 : this.getHistogram()[i];
			distance+= this.getHistogram()[i] - h2.getHistogram()[i] * Math.log10(numerator/divider);
		}
		return distance;
	}
	

	public int[] getHistogram() {
		return histogram;
	}

	public void setHistogram(int[] histogram) {
		this.histogram = histogram;
	}

	public int getBins() {
		return bins;
	}

	public void setBins(int bins) {
		this.bins = bins;
	}
	
	public Histogram addHistograms(Histogram h2) {
		Histogram result = new Histogram(h2.getBins());

		for (int i = 0; i < h2.bins; i++) {
			result.histogram[i] = this.histogram[i] + h2.histogram[i];
		}
		return result;
	}

	public Histogram multiplyHistograms(double multiplier) {
		Histogram result = new Histogram(this.bins);

		for (int i = 0; i < this.bins; i++) {
			result.histogram[i] = (int) (this.histogram[i] * multiplier);
		}
		return result;
	}
	
	private double calculateStandardDeviation() {
		double mean = calculateMean();
		
		double sum = 0.0;
		
		for(int i = 0; i < bins; i++){
			sum += Math.pow(histogram[i] - mean, 2);
		}
		
		return sum/numberData();
	}
	
	private double calculateMean(){
		double sum = 0.0;
		
		for(int i = 0; i < bins; i++){
			sum += histogram[i] * i;
		}		
		return sum/numberData();
	}
	
	private int numberData(){
		int number = 0;
		
		for(int i = 0; i < bins; i++){
			number += histogram[i];
		}
		return number;
	}

	public double getStandardDeviation() {
		return Math.sqrt(calculateStandardDeviation());
	}

}
