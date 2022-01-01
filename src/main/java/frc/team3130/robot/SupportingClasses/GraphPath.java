package frc.team3130.robot.SupportingClasses;

import java.util.ArrayDeque;
import java.util.ArrayList;

public class GraphPath implements Comparable<GraphPath>{
    private double distance;
    private final ArrayDeque<Node> path;

    public GraphPath(double distance, ArrayDeque<Node> path) {
        this.distance = distance;
        this.path = new ArrayDeque<>();
        this.path.addAll(path);
    }

    public GraphPath(double distance) {
        this(distance, new ArrayDeque<>());
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

    public ArrayDeque<Node> getPath() {
        return path;
    }

    public void addNodeToPath(Node nextNode) {
        path.add(nextNode);
    }

    public void addNodeToPath(Node nextNode, double distance) {
        addNodeToPath(nextNode);
        addDistance(distance);
    }

    public int getSteps() {
        return path.size();
    }

    public GraphPath copy() {
        return new GraphPath(distance, new ArrayDeque<>(path));
    }

    public String toString() {
        return "path: " + path;
    }

    @Override
    public int compareTo(GraphPath other) {
        return Double.compare(distance, other.distance);
    }
}
