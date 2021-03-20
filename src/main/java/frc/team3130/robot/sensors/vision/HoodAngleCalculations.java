package frc.team3130.robot.sensors.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.util.LinearInterp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

public class HoodAngleCalculations {

//    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");
//
//    private NetworkTableEntry calcAngle =
//            tab.add("Calculated angle", 0.0).getEntry();

    private static Comparator<DataPoint> compPoint = new Comparator<DataPoint>() {
        @Override
        public int compare(DataPoint p1, DataPoint p2) {
            if (p1.greaterThan(p2))
                return 1;
            else if (p1.lessThan(p2))
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
            if (point.contains(",")) {
                String[] parts = point.split(",");
                distance = Double.parseDouble(parts[0]);
                speed = Double.parseDouble(parts[1]);
            }
        }

        @Override
        public String toString() {
            return "(" + distance + "," + speed + ")";
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
    private static HoodAngleCalculations m_pInstance;

    public static HoodAngleCalculations GetInstance() {
        if (m_pInstance == null) m_pInstance = new HoodAngleCalculations();
        return m_pInstance;
    }

    private ArrayList<DataPoint> data_MainStorage;
    private LinearInterp speedCurve;
    private final String FILEPATH;


    private static double AngleOffset;

    public double getRPMOffset() {
        return AngleOffset;
    }


    public HoodAngleCalculations() {
        FILEPATH = Filesystem.getDeployDirectory() + File.separator + "hood_angle_data.csv";


        AngleOffset = RobotMap.kAngleChange;

        data_MainStorage = new ArrayList<DataPoint>();
        readFile();
        speedCurve = null;
        loadCurve();
    }

    public void loadCurve() {
        ArrayList<Double> data_Dist = new ArrayList<Double>();
        ArrayList<Double> data_Speed = new ArrayList<Double>();

        for (int iii = 0; iii < data_MainStorage.size(); iii++) {
            DataPoint pt = data_MainStorage.get(iii);
            data_Dist.add(pt.distance);
            data_Speed.add(pt.speed);
        }

        speedCurve = new LinearInterp(data_Dist, data_Speed);
    }

    public void readFile() {
        data_MainStorage.clear();

        try (BufferedReader br = new BufferedReader(new FileReader(FILEPATH))) {
            for (String line; (line = br.readLine()) != null; ) {
                if (!line.equals("")) data_MainStorage.add(new DataPoint(line));
            }
            // line is not visible here.
        } catch (IOException e) {
            e.printStackTrace();
        }

        data_MainStorage.sort(compPoint);
        loadCurve();
    }


    public void incrementAngleOffset() {
        AngleOffset += 0.05;
    }

    public void decrementAngleOffset() {
        AngleOffset -= 0.05;
    }

    public void resetAngleOffset(){
        AngleOffset = RobotMap.kRPMChange;
    }

    public double getAngle(Double dist) {
        double speed = speedCurve.getY(dist);
        return (speed + speed * getRPMOffset());
    }

    public void outputToShuffleboard() {
//        SmartDashboard.putNumber("", HoodAngleCalculations.GetInstance().getAngle(calcAngle.getDouble(0)));
    }
}

