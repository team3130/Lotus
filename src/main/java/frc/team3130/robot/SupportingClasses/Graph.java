package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.*;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private final ArrayList<Node> nodes;

    // adjacency matrix
    private double[][][] graph;

    // used for ensuring it's a singleton
    private static final Graph m_instance = new Graph();

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
        graph = new double[3][3][3];

        // O(n^3)
        for (int i = 0; i < graph.length; i++) {
            for (int j = 0; j < graph.length; j++) {
                for (int k = 0; k < graph.length; k++) {
                    graph[i][j][k] = 0;
                }
            }
        }

        nodes.add(new Node(1, 1));
        nodeMap.put(nodes.get(0), 0);
    }

    /**
     * To be called by connectNode in a for loop
     * @param toBeAdded node that will be added (the one added most recently)
     * @param ConnectedTo node that it will be connected to
     */
    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo) {
        double distance = toBeAdded.getDistance(ConnectedTo);
        for (int looper = 0; looper < nodes.size(); looper++) {
            // logistical equation to care about turning more if the ball is closer
            double weight = distance + Math.abs((Math.abs(toBeAdded.getAngleToFrom(ConnectedTo, nodes.get(looper))) / 35) * (-1 * (1 / (1 + 10 * Math.pow(Math.E, (-0.7 * distance)))) + 1));
            graph[looper][nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = weight;
            graph[looper][nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = weight;
        }
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

    public boolean containsDuplicates() {
        Set<Node> seen = new HashSet<>();
        for (Node node : nodes) {
            if (seen.contains(node)) {
                return true;
            }
            else {
                seen.add(node);
            }
        }
        return false;
    }

    /**
     * Connects the node to every node in the graph
     * @param newNode the node to be connected
     */
    private void ConnectNode(Node newNode) {
        for (int i = 1; i < nodes.size(); i++) {
            // checks to make sure it doesn't add itself
            // we wouldn't need this if we just didn't iterate to the last element however threading issues could occur
            if (nodes.get(i) != newNode) {
                // basically the A* heuristic that says don't go backwards
                if (!(newNode.getAngleToFrom(nodes.get(i), nodes.get(0)) < 0)) {
                    putNodeInGraph(newNode, nodes.get(i));
                }
            }
        }
    }

    private void resize() {
        double[][][] newMatrix = new double[nodes.size()][nodes.size()][nodes.size()];
        // iterate through matrix and set the values in newMatrix to the old ones O(n^3)
        for (int i = 0; i < graph.length; i++) {
            for (int j = 0; j < graph.length; j++) {
                for (int k = 0; k < graph.length; k++){
                    // check if the index is in bounds
                    if (i < newMatrix.length && j < newMatrix.length && k < newMatrix.length) {
                        newMatrix[i][j][k] = graph[i][j][k];
                    }
                }
            }
        }
        graph = newMatrix;
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

            // selects the matrix with the one from the previous
            double[][] matrix = graph[nodeMap.get(tempPath.getPath().get(tempPath.getPath().size() - 2))];

            // caching the index
            int indexOfCurr = nodeMap.get(curr);

            // lets the algorithm know that we have visited this node
            visited[indexOfCurr] = true;

            // should ensure that we find a 5-step path if one exists
            if (goal == curr) {
                System.out.println("Exiting from first if statement !!!!! ");
                return tempPath;
            }

            // the row that corresponds to the node
            double[] adj = matrix[indexOfCurr];

            // for each adjacent node
            for (int looper = 0; looper < adj.length; looper++) {
                if (!visited[looper] && (tempPath.getDistance() + matrix[looper][indexOfCurr] < distances[looper]) && (matrix[looper][indexOfCurr] != 0 && (tempPath.getSteps() == steps - 1 || !(nodes.get(looper).equals(goal))))) {
                    distances[looper] = tempPath.getDistance() + matrix[indexOfCurr][looper];
                    tempPath.addDistance(matrix[indexOfCurr][looper]);
                    tempPath.addNodeToPath(nodes.get(looper));
                    queue.add(tempPath);
                }
            }
            data = tempPath;
        }
        System.out.println("Exiting because ran out of time !!!!! ");
        System.out.println("size: " + data.getPath().size());
        return data;
    }

    public void printGraph() {
        for (Node node : nodes) {
            System.out.print("(" + node.x_pos + ", " + node.y_pos + "), ");
        }
        System.out.print("\n");

        for (double[][] matrix : graph) {
            for (double[] doubles : matrix) {
                for (double aDouble : doubles) {
                    System.out.print(aDouble + ", ");
                }
                System.out.print("\n");
            }
        }
    }

    public boolean containsNan() {
        boolean toBeReturned = false;
        for (double[][] doubles : graph) {
            for (int j = 0; j < graph.length; j++) {
                for (int k = 0; k < graph.length; k++) {
                    if (Double.isNaN(doubles[j][k])) {
                        toBeReturned = true;
                        break;
                    }
                }
            }
        }
        return toBeReturned;
    }
}
