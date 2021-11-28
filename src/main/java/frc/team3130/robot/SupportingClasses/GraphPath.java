package frc.team3130.robot.SupportingClasses;

import java.util.ArrayList;

public class GraphPath {
    private double distance;
    private ArrayList<Node> path;

    public GraphPath(double distance, ArrayList<Node> path) {
        this.distance = distance;
        this.path = path;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void addDistance(double distance) {
        this.distance += distance;
    }

    public ArrayList<Node> getPath() {
        return path;
    }

    public int getSteps() {
        return path.size();
    }

    public GraphPath copy() {
        GraphPath newOne = new GraphPath(distance, new ArrayList<>(path));
        return newOne;
    }

}
