package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import javax.swing.*;
import java.util.*;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private ArrayList<Node> nodes;
    private double[][] matrix;

    private static Graph m_instance = new Graph();

    public static Graph getInstance() {
        return m_instance;
    }
    
    private Graph() {
        nodes = new ArrayList<>();
        nodeMap = new HashMap<>();
        matrix = new double[3][3];

        for (int j = 0; j < matrix.length - 1; j++) {
            for (int k = 0; k < matrix.length - 1; k++) {
                matrix[j][k] = 0;
            }
        }

        nodes.add(new Node(Chassis.getInstance().getPosCV().getX(), Chassis.getInstance().getPosCV().getY()));
        nodeMap.put(nodes.get(0), 0);
    }

    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo) {
        matrix[nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = toBeAdded.getDistance(ConnectedTo);
        matrix[nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = toBeAdded.getDistance(ConnectedTo);
    }

    public void addNode(Node newNode) {
        nodes.add(newNode);
        nodeMap.put(newNode, nodes.size() - 1);
        resize();
        ConnectNode(newNode);
    }

    public boolean contains(Node comparator) {
        return nodeMap.containsKey(comparator);
    }

    private void ConnectNode(Node newNode) {
        for (int i = 0; i < nodes.size(); i++) {
            if (nodes.get(i) != newNode) {
                putNodeInGraph(newNode, nodes.get(i));
            }
        }
    }

    private void resize() {
        double[][] newMatrix = new double[nodes.size()][nodes.size()];
        // iterate through matrix and set the values in newMatrix to the old ones
        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix.length; j++) {
                // check if the index is in bounds
                if (i < newMatrix.length && j < newMatrix.length) {
                    newMatrix[i][j] = matrix[i][j];
                }
            }
        }
        matrix = newMatrix;
    }

    public ArrayList<Node> getPath(int steps) {
        GraphPath winner = new GraphPath(Double.MAX_VALUE, new ArrayList<>());
        // for each element of nodes except for the bot which is at index 0
        for (int i = 1; i < nodes.size(); i++) {
            GraphPath first = Dijkstra(nodes.get(i), steps);
            if (first.getSteps() == steps && first.getDistance() < winner.getDistance()) {
                winner = first;
            }
        }
        return winner.getPath();
    }
        
    private ArrayList<Node> getAdj(Node next, Node goal, GraphPath gpath, int steps) {
        ArrayList<Node> adjacent = new ArrayList<>();
        int index = nodeMap.get(next);

        for (int looper = 0; looper < matrix.length; looper++) {
            if (matrix[index][looper] != 0) {
                if (gpath.getSteps() == steps - 1) {
                    adjacent.add(nodes.get(looper));
                }
                else {
                    if (nodes.get(looper) != goal) {
                        adjacent.add(nodes.get(looper));
                    }
                }
            }
        }
        return adjacent;
    }

    public GraphPath Dijkstra(Node goal, int steps) {
        ArrayList<Node> path = new ArrayList<>();
        GraphPath data = new GraphPath(0, path);

        boolean[] visited = new boolean[nodes.size()];
        double[] distances = new double[nodes.size()];

        Arrays.fill(distances, Double.MAX_VALUE);

        distances[0] = 0;

        PriorityQueue<GraphPath> queue = new PriorityQueue<>(Comparator.comparingDouble(GraphPath::getDistance));

        ArrayList<Node> temp = new ArrayList<>();
        temp.add(nodes.get(0));
        queue.add(new GraphPath(0, temp));

        while (!queue.isEmpty()) {
            GraphPath tempPath = queue.poll().copy(); // the copy is to prevent modifying only the same object
            Node curr = tempPath.getPath().get(tempPath.getPath().size() - 1);

            visited[nodeMap.get(curr)] = true;

            // should ensure that we find a 5-step path if one exists
            if (goal == curr && tempPath.getSteps() == steps) {
                return tempPath;
            }

            ArrayList<Node> adj = getAdj(curr, goal, tempPath, steps);

            for (int looper = 0; looper < adj.size(); looper++) {
                if (!visited[nodeMap.get(adj.get(looper))] && tempPath.getDistance() + matrix[nodeMap.get(adj.get(looper))][nodeMap.get(curr)] < distances[nodeMap.get(adj.get(looper))]) {
                    distances[nodeMap.get(adj.get(looper))] = tempPath.getDistance() + matrix[nodeMap.get(curr)][nodeMap.get(adj.get(looper))];
                    tempPath.addDistance(matrix[nodeMap.get(curr)][nodeMap.get(adj.get(looper))]);
                    tempPath.getPath().add(adj.get(looper));
                    queue.add(tempPath);
                }
            }
            data = tempPath;
        }
        return data;
    }
}
