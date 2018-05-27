package LargestRigidPart;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javafx.application.Application;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.chart.BarChart;
import javafx.scene.chart.CategoryAxis;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Label;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

public class HistogramPlot extends Application {

	int DATA_SIZE = 1000;
	Stage primaryStage;
	
	@Override
	public void start(Stage primaryStage) {
		this.primaryStage = primaryStage;
		prepareData();
	}

	private void drawHistogram(List<String> data, String title) {
		Label labelInfo = new Label();

		final CategoryAxis xAxis = new CategoryAxis();
		final NumberAxis yAxis = new NumberAxis();
		final BarChart<String, Number> barChart = new BarChart<>(xAxis, yAxis);
		barChart.setCategoryGap(0);
		barChart.setBarGap(0);

		// xAxis.setLabel("Range");
		// yAxis.setLabel("Population");

		XYChart.Series series1 = new XYChart.Series();
		
		int i = 0;
		
		for(String dataset : data){
			if(!dataset.equals("")){
				series1.getData().add(new XYChart.Data(String.valueOf(i++), Integer.parseInt(dataset)));
			}
		}

		barChart.getData().addAll(series1);
		barChart.setLegendVisible(false);

		for (Node n : barChart.lookupAll(".default-color0.chart-bar")) {
			n.setStyle("-fx-bar-fill: grey;");
		}

		VBox vBox = new VBox();
		vBox.getChildren().addAll(labelInfo, barChart);

		StackPane root = new StackPane();
		root.getChildren().add(vBox);

		Scene scene = new Scene(root, 500, 450);

		primaryStage.setTitle("Feature histogram for " + title);
		primaryStage.setScene(scene);
		primaryStage.show();
	}

	public static void main(String[] args) {
		launch(args);
	}

	// generate dummy random data
	private void prepareData() {
		URL path = HistogramPlot.class.getResource("histogram.txt");
		String line;
		
		List<String> histograms;
		List<String> histogram1;
		List<String> histogram2;

		try {
			BufferedReader bufferreader = new BufferedReader(new FileReader(path.getFile()));
			int number = 1;
//			while ((line = bufferreader.readLine()) != null) {
				line = bufferreader.readLine();
				histograms  = Arrays.asList(line.split(","));
				histogram1 = Arrays.asList(histograms.get(0).split(" "));
				histogram2 = Arrays.asList(histograms.get(1).split(" "));
				
				drawHistogram(histogram1, "reference " + number);
				drawHistogram(histogram2, "target " + number++);
//			}
		} catch (FileNotFoundException ex) {
			ex.printStackTrace();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
	}
}
