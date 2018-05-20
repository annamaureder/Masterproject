package LargestRigidPart;

import java.awt.Color;

import ij.gui.Plot;
import ij.gui.PlotWindow;
import ij.plugin.PlugIn;
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
	
	public void meanHistogram(){
		
	}
	
	public boolean matches(Histogram h2){
		return false;
		
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
