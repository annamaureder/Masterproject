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
	
//	public void showHistogram(){
//		
//		// Create histogram from data
//		@SuppressWarnings("unchecked")
//		DataTable data = new DataTable(Double.class);
//		for (int i = 0; i < histogram.length; i++) {
//			for(int j = 0; j < histogram[i]; j++){
//				data.add(i);
//			}
//		}
//				Histogram1D histogram = new Histogram1D(data, Orientation.VERTICAL,
//						new Number[] {0, 1, 2, 3, 4, 5, 6, 7, 8});
//				// Create a second dimension (x axis) for plotting
//				DataSource histogram2d = new EnumeratedData(histogram, (-4.0 + -3.2)/2.0, 0.8);
//
//				// Create new bar plot
//				BarPlot plot = new BarPlot(histogram2d);
//
//				// Format plot
//				plot.setInsets(new Insets2D.Double(20.0, 65.0, 50.0, 40.0));
//				plot.getTitle().setText(
//						String.format("Distribution of %d random samples", data.getRowCount()));
//				plot.setBarWidth(0.78);
//
//				// Format x axis
//				plot.getAxisRenderer(BarPlot.AXIS_X).setTickAlignment(0.0);
//				plot.getAxisRenderer(BarPlot.AXIS_X).setTickSpacing(0.8);
//				plot.getAxisRenderer(BarPlot.AXIS_X).setMinorTicksVisible(false);
//				// Format y axis
//				plot.getAxis(BarPlot.AXIS_Y).setRange(0.0,
//						MathUtils.ceil(histogram.getStatistics().get(Statistics.MAX)*1.1, 25.0));
//				plot.getAxisRenderer(BarPlot.AXIS_Y).setTickAlignment(0.0);
//				plot.getAxisRenderer(BarPlot.AXIS_Y).setMinorTicksVisible(false);
//				plot.getAxisRenderer(BarPlot.AXIS_Y).setIntersection(-4.4);
//
//				// Format bars
//				plot.getPointRenderer(histogram2d).setColor(
//					GraphicsUtils.deriveWithAlpha(Color.red, 128));
//				plot.getPointRenderer(histogram2d).setValueVisible(true);
//
//				// Add plot to Swing component
//				InteractivePanel panel = new InteractivePanel(plot);
//				panel.setPannable(false);
//				panel.setZoomable(false);
//				add(panel);
//				
//		//this.showInFrame();
//	}
	
	public void showTestData(){
        float[] x = new float[bins];
        float[] y = new float[bins];
        for(int i = 0; i < bins; i++){
        	x[i] = i;
        	y[i] = histogram[i];
        }

        PlotWindow.noGridLines = false; // draw grid lines
        Plot plot = new Plot("Feature histograms","bins","",x,y);
        plot.setLimits(0, bins, 0, 10);
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
}
