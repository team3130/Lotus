package frc.team3130.robot.SupportingClasses;

import java.lang.reflect.Array;
import java.util.*;
import java.util.function.BiFunction;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    public final ArrayList<Node> nodes;

    // adjacency matrix
    private double[][][] adjTensor;

    // lambda for logistical equation
    private final BiFunction<Double, Double, Double> logEquation = (Double angle, Double distance) -> (angle * (-1 * (1 / (1 + 10 * Math.pow(Math.E, (-0.7 * distance)))) + 1));

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
        adjTensor = new double[3][3][3];

        // O(n^3)
        for (int i = 0; i < adjTensor.length; i++) {
            for (int j = 0; j < adjTensor.length; j++) {
                for (int k = 0; k < adjTensor.length; k++) {
                    adjTensor[i][j][k] = 0;
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
            double weight = distance + logEquation.apply(Math.abs(toBeAdded.getAngleToFrom(ConnectedTo, nodes.get(looper))) / 35, distance);
            // add to directed graph
            adjTensor[looper][nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = weight;
            adjTensor[looper][nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = weight;
        }
    }

    /**
     * Designed to connect nodes in the graph that involve the bot
     * instead of using getAngleToForm we instead use the
     * @param botNode the node that corresponds to nodes.get(0) aka the bot
     * @param ConnectedTo the node connecting to
     */
    private void putNodeInGraphConnectedToBot(Node botNode, Node ConnectedTo) {
        //TODO: replace botAngle with Chassis.getInstance().getHeading()
        double botAngle = 0;
        // distance
        double distance = botNode.getDistance(ConnectedTo);
        // weight using the logistical equation
        double weight = distance + logEquation.apply(Math.abs((botNode.getRelAngle(ConnectedTo) + botAngle) / 20), distance);
        // add to every matrix in the tensor
        for (int looper = 0; looper < nodes.size(); looper++) {
            // add to undirected graph
            adjTensor[looper][nodeMap.get(botNode)][nodeMap.get(ConnectedTo)] = weight;
            adjTensor[looper][nodeMap.get(ConnectedTo)][nodeMap.get(botNode)] = weight;
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

    /**
     * checks if the node exists
     * @param comparator the node that is being checked
     * @return if the node is registered
     */
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
        for (int i = 0; i < nodes.size(); i++) {
            // checks to make sure it doesn't add itself
            // we wouldn't need this if we just didn't iterate to the last element however threading issues could occur
            if (nodes.get(i) != newNode && i != 0) {
                putNodeInGraph(newNode, nodes.get(i));
            }
            // will run if new Node is not current and i == 0;
            else if (nodes.get(i) != newNode && i == 0) {
                // connect off of distance
                putNodeInGraphConnectedToBot(nodes.get(i), newNode);
            }
        }
    }

    private void resize() {
        double[][][] newMatrix = new double[nodes.size()][nodes.size()][nodes.size()];
        // iterate through matrix and set the values in newMatrix to the old ones O(n^3)
        for (int i = 0; i < adjTensor.length; i++) {
            for (int j = 0; j < adjTensor.length; j++) {
                for (int k = 0; k < adjTensor.length; k++){
                    // check if the index is in bounds
                    if (i < newMatrix.length && j < newMatrix.length && k < newMatrix.length) {
                        newMatrix[i][j][k] = adjTensor[i][j][k];
                    }
                }
            }
        }
        adjTensor = newMatrix;
    }

    /**
     * Uses Dijkstra's algorithm to determine the best path to take
     * @param steps the number of nodes that you want the ball to go through
     * @return the shortest path to a node
     */
    public ArrayDeque<Node> getPath(int steps) {
        GraphPath winner = new GraphPath(Double.MAX_VALUE, new ArrayDeque<>());
        // for each element of nodes except for the bot which is at index 0
        for (int i = 1; i < nodes.size(); i++) {
            GraphPath first = Dijkstra(nodes.get(i), steps);
            System.out.println("first: " + first);
            System.out.println("goal: " + nodes.get(i));
            if (first.getDistance() < winner.getDistance()) {
                winner = first;
            }
        }
        return winner.getPath();
    }

    public int factorial(int n) {
        if (n <= 0) {
            return 1;
        }
        return n * factorial(--n);
    }

    /**
     * brute force find the best path by generating every possible path of size steps
     * this is a rather slow solution
     * @param steps the size of the path that will be generated
     * @return the fastest path
     */
/*    public GraphPath bruteForce(int steps) {
        if (nodes.isEmpty()) {
            return new GraphPath(0, new ArrayDeque<>());
        }
        Node[] nodeArray = new Node[nodes.size() - 1];

        int amntOfObjects = factorial(nodeArray.length);
        int sample = factorial(nodeArray.length - steps);
        int combinations = amntOfObjects / sample;

        GraphPath shortest = new GraphPath(Double.MAX_VALUE, new ArrayDeque<>());

        // generate all possible paths
        for (int bigLooper = 0; bigLooper < combinations; bigLooper++) {
            for (int looperSwapper1 = 0; looperSwapper1 < nodeArray.length; looperSwapper1++) {
                for (int looperSwapper2 = 0; looperSwapper2 < nodeArray.length; looperSwapper2++) {
                    GraphPath temp = new GraphPath(0, );
                }
            }
        }

    }*/


    /**
     * Initial call for permutation
     * @return the shortest path
     */
    public GraphPath permute() {
        // nodes without the first node because that is the bot
        Node[] nodeses = new Node[nodes.size() - 1];
        // for looper to add every item except for the 0th index in nodes
        for (int looper = 1; looper < nodes.size(); looper++) {
            nodeses[looper - 1] = nodes.get(looper);
        }
        // "Array" of the shortest Graph paths should always be of size one
        GraphPath[] shortest = new GraphPath[] {new GraphPath(Double.MAX_VALUE)};
        // initial call to the recursive function
        Permutation(0, nodeses, shortest);
        // because of the array we don't need to worry about the method returning anything
        return shortest[0];
    }

    /**
     * Recursive method to permute through all possible ones
     * @param index1 index used in method, pass in 0 for initial call
     * @param nodes the array of the sample nodes
     * @param shortest an array of size one to pass the shortest array by reference, because java sucks
     */
    private void Permutation(int index1, Node[] nodes, GraphPath[] shortest) {
        // base case that checks if we have iterated to the end of the list
        if (index1 != nodes.length - 1) {
            // for loop
            for (int index2 = index1, l = nodes.length; index2 < l; index2++) {
                // swap two indices to permute through the array using heaps algorithm
                swap(nodes, index1, index2);
                // store the distance in a double variable for comparison
                double tempDistance = determineDistance(nodes);
                // check if the distance is less than the shortest one seen so far
                if (tempDistance < shortest[0].getDistance()) {
                    // temporary graph path that reflects the current node
                    GraphPath temp = new GraphPath(0);
                    // iterate to add items to the temporary GraphPath
                    for (Node node : nodes)  temp.addNodeToPath(node);
                    // determine the distance that this path takes
                    temp.setDistance(tempDistance);
                    shortest[0] = temp;
                }
                // re-call the method here
                Permutation(index1 + 1, nodes, shortest);
                // un-swap the array at the end of each iteration of the for looper
                swap(nodes, index1, index2);
            }
        }
    }

    /**
     * Determines the "distance" traveled
     * Summation of the weights
     * @param arr the path combination
     * @return the distance traveled
     */
    private double determineDistance(Node[] arr) {
        // the first one is going to the first node and the second one is getting to the second node
        // remember if all of them are wrong by a similar amount then it doesn't matter too much I hope
        double distanceTemp = adjTensor[0][0][nodeMap.get(arr[0])] + adjTensor[0][nodeMap.get(arr[0])][nodeMap.get(arr[1])];
        // iterate through array except for the first and last item
        for (int looper = 2; looper < arr.length; looper++) {
            // add distance of getting to the looper node from using the past two
            distanceTemp += adjTensor[nodeMap.get(arr[looper - 2])][nodeMap.get(arr[looper - 1])][nodeMap.get(arr[looper])];
        }
        // return the distance
        return distanceTemp;
    }

    /**
     * Swaps items in a given Node Array
     * @param arr array that the values get swapped in
     * @param index1 the index of the first element to be swapped
     * @param index2 the index of the second element to be swapped
     */
    private void swap(Node[] arr, int index1, int index2) {
        Node temp = arr[index1];
        arr[index1] = arr[index2];
        arr[index2] = temp;
    }

    /**
     * An implementation of Dijkstra's algorithm
     * @param goal the Node searching for
     * @param steps the desired amount of balls to collect in the path
     * @return the path in a {@link GraphPath} object
     */
    public GraphPath Dijkstra(Node goal, int steps) {
        ArrayDeque<Node> path = new ArrayDeque<>();
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

        ArrayDeque<Node> temp = new ArrayDeque<>();
        temp.add(nodes.get(0));
        queue.add(new GraphPath(0, temp));

        while (!queue.isEmpty()) {
            // makes a copy of the GraphPath object from the min heap
            GraphPath tempPath = queue.poll().copy(); // the copy is to prevent modifying only the same object
            // sets the current node to the one that was added to the path most recently
            Node curr = tempPath.getPath().getLast();

            System.out.println(goal);
            System.out.println(queue);

            double[][] matrix;

            if (tempPath.getPath().size() > 2) {
                // temporary node removed from the back to be added back
                Node temptemp = tempPath.getPath().pollLast();
                // selects the matrix with the one from the previous
                matrix = adjTensor[nodeMap.get(tempPath.getPath().getLast())];
                // add the node back to the back of the deque
                tempPath.getPath().add(temptemp);
            }

            else {
                matrix = adjTensor[0];
            }

            // caching the index
            int indexOfCurr = nodeMap.get(curr);

            // lets the algorithm know that we have visited this node
            visited[indexOfCurr] = true;

            // should ensure that we find a 5-step path if one exists
            if (goal.equals(curr)) {
                System.out.println("exiting from thing");
                return tempPath;
            }

            // the row that corresponds to the node
            double[] adj = matrix[indexOfCurr];

            // for each adjacent node
            for (int looper = 0; looper < adj.length; looper++) {
                GraphPath tempIteratorPath = tempPath.copy();
                // additional logic: && (tempPath.getSteps() == steps - 1 || !(nodes.get(looper).equals(goal))))
                boolean one = !visited[looper];
                boolean two = (tempPath.getDistance() + adj[looper] < distances[looper]);
                boolean three = (adj[looper] != 0);
                boolean four = tempPath.getSteps() == steps - 1;
                boolean five = !(nodes.get(looper).equals(goal));

                System.out.println("\n\nlooped one: " + nodes.get(looper));
                System.out.println("goal: " + goal);
                System.out.println("curr: " + curr);
                System.out.println("one: " + one);
                System.out.println("two: " + two);
                System.out.println("three: " + three);
                System.out.println("four: " + four);
                System.out.println("five: " + five + "\n\n");

                if (one && two && three && (five || four)) {
                    distances[looper] = tempIteratorPath.getDistance() + adj[looper];
                    tempIteratorPath.addDistance(adj[looper]);
                    tempIteratorPath.addNodeToPath(nodes.get(looper));
                    queue.add(tempIteratorPath);
                }
            }
            data = tempPath;
        }
        System.out.println("ran out of time");
        return data;
    }

    public void printGraph() {
        for (Node node : nodes) {
            System.out.print("(" + node.x_pos + ", " + node.y_pos + "), ");
        }
        System.out.print("\n");

        for (double[][] matrix : adjTensor) {
            for (double[] doubles : matrix) {
                for (double aDouble : doubles) {
                    System.out.print((int) aDouble + ", ");
                }
                System.out.print("\n");
            }
            System.out.print("\n");
        }
    }

    public boolean containsNan() {
        boolean toBeReturned = false;
        for (double[][] doubles : adjTensor) {
            for (int j = 0; j < adjTensor.length; j++) {
                for (int k = 0; k < adjTensor.length; k++) {
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
