package frc.team3130.robot.SupportingClasses;

import java.util.ArrayList;

public class GraphPath {
    private double distance;
    private final ArrayList<Node> path;
    private double currentHeading;

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
        path.add(nextNode);
        // we can get away with this because there will always be at least two elements in the array: the bot and the added node
        currentHeading += path.get(path.size() - 2).getRelAngle(path.get(path.size() - 1));
    }

    public void testHeading() {
        double temp = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            temp += path.get(i).getRelAngle(path.get(i + 1));
        }
        assert temp == currentHeading;
    }

    public int getSteps() {
        return path.size();
    }

    public GraphPath copy() {
        return new GraphPath(distance, new ArrayList<>(path));
    }

}
