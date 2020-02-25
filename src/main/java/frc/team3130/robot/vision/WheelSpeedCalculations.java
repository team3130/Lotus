package frc.team3130.robot.vision;

import edu.wpi.first.wpilibj.Filesystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.util.LinearInterp;

import java.io.*;
import java.util.ArrayList;
import java.util.Comparator;

public class WheelSpeedCalculations {
	
	private static Comparator<DataPoint> compPoint = new Comparator<DataPoint>() {
		@Override
		public int compare(DataPoint p1, DataPoint p2) {
			if(p1.greaterThan(p2))
				return 1;
			else if(p1.lessThan(p2))
				return -1;
			return 0;
		}
	};
	
	private static class DataPoint {
		double distance;
		double speed;
		
		public DataPoint(double dist, double speed) {
			distance = dist;
			this.speed = speed;
		}
		
		public DataPoint(String point) {
			if(point.contains(",")) {
				String[] parts = point.split(",");
				distance = Double.parseDouble(parts[0]);
				speed = Double.parseDouble(parts[1]);
			}
		}
		
		@Override
		public String toString() {
			return "("+distance+","+speed+")";
		}
		
		public boolean greaterThan(DataPoint other) {
			return distance > other.distance;
		}
		
		public boolean lessThan(DataPoint other) {
			return distance < other.distance;
		}
		
		@SuppressWarnings("unused")
		public boolean equals(DataPoint other) {
			return distance == other.distance;
		}
	}

	//Instance Handling
	private static WheelSpeedCalculations m_pInstance;

	public static WheelSpeedCalculations GetInstance() {
		if (m_pInstance == null) m_pInstance = new WheelSpeedCalculations();
		return m_pInstance;
	}

	private ArrayList<DataPoint> data_MainStorage;
	private LinearInterp speedCurve;
	private final String FILEPATH;

	public WheelSpeedCalculations() {
		if(RobotMap.kUseCompbot) {
			FILEPATH = Filesystem.getDeployDirectory() + File.separator +  "shooter_data_comp.csv";
		} else {
			FILEPATH = Filesystem.getDeployDirectory() + File.separator +  "shooter_data_practice.csv";
		}

		data_MainStorage = new ArrayList<DataPoint>();
		readFile();
		speedCurve = null;
		loadCurve();
	}
 	
	public void loadCurve() {
		ArrayList<Double> data_Dist = new ArrayList<Double>();
		ArrayList<Double> data_Speed = new ArrayList<Double>();
		
		for(int iii = 0; iii < data_MainStorage.size(); iii++){
			DataPoint pt = data_MainStorage.get(iii);
			data_Dist.add(pt.distance);
			data_Speed.add(pt.speed);
		}
		
		speedCurve = new LinearInterp(data_Dist, data_Speed);
	}
	
	public void readFile() {
		data_MainStorage.clear();
		
		try(BufferedReader br = new BufferedReader(new FileReader(FILEPATH))) {
		    for(String line; (line = br.readLine()) != null; ) {
		        if(!line.equals(""))data_MainStorage.add(new DataPoint(line));
		    }
		    // line is not visible here.
		}catch (IOException e) {
			e.printStackTrace();
		}

		data_MainStorage.sort(compPoint);
		loadCurve();
	}

	public double getSpeed(Double dist) {
		return speedCurve.getY(dist);
	}
}
