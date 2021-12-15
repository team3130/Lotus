package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import javax.swing.*;
import java.util.*;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private ArrayList<Node> nodes;

    // adjacency matrix
    private double[][] matrix;

    // used for ensuring it's a singleton
    private static Graph m_instance = new Graph();

    /**
     * @return the single instance for this class
     */
    public static Graph getInstance() {
        return m_instance;
    }

    /**
     * The only constructor of graph to be called once on robotInit
     */
    private Graph() {
        nodes = new ArrayList<>();
        nodeMap = new HashMap<>();
        matrix = new double[3][3];

        for (int j = 0; j < matrix.length - 1; j++) {
            for (int k = 0; k < matrix.length - 1; k++) {
                matrix[j][k] = 0;
            }
        }

        nodes.add(new Node(1, 1));
        nodeMap.put(nodes.get(0), 0);
    }

    /**
     * To be called by connectNode in a for loop
     * @param toBeAdded node that will be added
     * @param ConnectedTo node that it will be connected to
     */
    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo) {
        matrix[nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = toBeAdded.getDistance(ConnectedTo);
        matrix[nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = toBeAdded.getDistance(ConnectedTo);
    }

    /**
     * add newNode to the arrayList, put it, and it's index in a map, resize the matrix, and connect the nodes
     * @param newNode node that will be added to the graph
     */
    public void addNode(Node newNode) {
        nodes.add(newNode);
        nodeMap.put(newNode, nodes.size() - 1);
        resize();
        ConnectNode(newNode);
    }

    public boolean contains(Node comparator) {
        return nodeMap.containsKey(comparator);
    }

    /**
     * Connects the node to every node in the graph
     * @param newNode the node to be connected
     */
    private void ConnectNode(Node newNode) {
        for (Node node : nodes) {
            // checks to make sure it doesn't add itself
            // we wouldn't need this if we just didn't iterate to the last element however threading issues could occur
            if (node != newNode) {
                putNodeInGraph(newNode, node);
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

    /**
     * An implementation of Dijkstra's algorithm
     * @param goal the Node searching for
     * @param steps the desired amount of balls to collect in the path
     * @return the path in a {@link GraphPath} object
     */
    public GraphPath Dijkstra(Node goal, int steps) {
        ArrayList<Node> path = new ArrayList<>();
        GraphPath data = new GraphPath(0, path);

        boolean[] visited = new boolean[nodes.size()];
        double[] distances = new double[nodes.size()];

        // fills the array with false to show that it has not been visited
        Arrays.fill(visited, false);
        // sets the distance to the max value so that the first path will be smaller then nothing
        Arrays.fill(distances, Double.MAX_VALUE);

        // sets the distance to the bot to be zero
        distances[0] = 0;

        PriorityQueue<GraphPath> queue = new PriorityQueue<>(Comparator.comparingDouble(GraphPath::getDistance));

        ArrayList<Node> temp = new ArrayList<>();
        temp.add(nodes.get(0));
        queue.add(new GraphPath(0, temp));

        while (!queue.isEmpty()) {
            // makes a copy of the GraphPath object from the min heap
            GraphPath tempPath = queue.poll().copy(); // the copy is to prevent modifying only the same object
            // sets the current node to the one that was added to the path most recently
            Node curr = tempPath.getPath().get(tempPath.getPath().size() - 1);

            // caching the index
            int indexOfCurr = nodeMap.get(curr);

            // lets the algorithm know that we have visited this node
            visited[indexOfCurr] = true;

            // should ensure that we find a 5-step path if one exists
            if (goal == curr && tempPath.getSteps() == steps) {
                return tempPath;
            }

            // the row that corresponds to the node
            double[] adj = matrix[indexOfCurr];

            // for each adjacent node
            for (int looper = 0; looper < adj.length; looper++) {
                if (!visited[looper] && (tempPath.getDistance() + matrix[looper][indexOfCurr] < distances[looper]) && (matrix[looper][indexOfCurr] != 0 && (tempPath.getSteps() == steps - 1 || !nodes.get(looper).equals(goal)))) {
                    distances[looper] = tempPath.getDistance() + matrix[indexOfCurr][looper];
                    tempPath.addDistance(matrix[indexOfCurr][looper]);
                    tempPath.addNodeToPath(nodes.get(looper));
                    queue.add(tempPath);
                }
            }
            data = tempPath;
        }
        return data;
    }
}
