package frc.team3130.robot.SupportingClasses;

import java.util.ArrayList;

public class GraphPath {
    private double distance;
    private final ArrayList<Node> path;

    public GraphPath(double distance, ArrayList<Node> path) {
        this.distance = distance;
        this.path = new ArrayList<>();
        this.path.addAll(path);
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

    public void addNodeToPath(Node nextNode) {
        System.out.println(nextNode);
        path.add(nextNode);
    }

    public int getSteps() {
        return path.size();
    }

    public GraphPath copy() {
        return new GraphPath(distance, new ArrayList<>(path));
    }

    public String toString() {
        return "path: " + path;
    }

}
