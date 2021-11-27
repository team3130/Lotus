package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.*;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private ArrayList<Node> nodes;
    private Double[][] matrix;

    public Graph() {
        nodes = new ArrayList<>();
        nodeMap = new HashMap<>();
        matrix = new Double[2][2];

        nodes.add(new Node(Chassis.getInstance().getPosCV().getX(), Chassis.getInstance().getPosCV().getY(), PhysicalObject.bot));
    }

    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo) {
        matrix[nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = toBeAdded.getDistance(ConnectedTo);
        matrix[nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = toBeAdded.getDistance(ConnectedTo);
    }

    public void addNode(Node newNode) {
        nodes.add(newNode);
        resize();
        ConnectNode(newNode);
    }

    private void ConnectNode(Node newNode) {
        for (int i = 0; i < nodes.size(); i++) {
            if (nodes.get(i) != newNode) {
                putNodeInGraph(newNode, nodes.get(i));
            }
        }
    }

    private void resize() {
        matrix = Arrays.copyOf(matrix, nodes.size());
    }

    public ArrayList<Node> getPath() {
        GraphPath winner = new GraphPath(Double.MAX_VALUE, new ArrayList<>());
        // for each element of nodes except for the bot which is at index 0
        for (int i = 1; i < nodes.size(); i++) {
            GraphPath first = Dijkstra(nodes.get(i));
            if (first.getSteps() == 5 && first.getDistance() < winner.getDistance()) {
                winner = first;
            }
        }
        return winner.getPath();
    }

    private ArrayList<Node> getAdj(Node next, Node goal, GraphPath gpath) {
        ArrayList<Node> adjacent = new ArrayList<>();
        int index = nodeMap.get(next);

        for (int looper = 0; looper < matrix.length; looper++) {
            if (matrix[index][looper] != 0) {
                if (gpath.getSteps() == 4) {
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

    public GraphPath Dijkstra(Node goal) {
        ArrayList<Node> path = new ArrayList<>();
        GraphPath data = new GraphPath(0, path);

        boolean[] visited = new boolean[nodes.size()];
        double[] distances = new double[nodes.size()];

        Arrays.fill(distances, Double.MAX_VALUE);

        distances[0] = 0;

        PriorityQueue<GraphPath> queue = new PriorityQueue<>(new Comparator<GraphPath>() {
            @Override
            public int compare(GraphPath o1, GraphPath o2) {
                return Double.compare(o1.getDistance(), o2.getDistance());
            }
        });

        ArrayList<Node> temp = new ArrayList<>();
        temp.add(nodes.get(0));
        queue.add(new GraphPath(0, temp));

        while (!queue.isEmpty()) {
            GraphPath tempPath = queue.poll().copy(); // the copy is to prevent modifying only the same object
            Node curr = tempPath.getPath().get(tempPath.getPath().size() - 1);

            visited[nodeMap.get(curr)] = true;

            // should ensure that we find a 5-step path if one exists
            if (goal == curr && tempPath.getSteps() == 5) {
                return tempPath;
            }

            ArrayList<Node> adj = getAdj(curr, goal, tempPath);

            for (int looper = 0; looper < adj.size(); looper++) {
                if (!visited[nodeMap.get(adj.get(looper))] && tempPath.getDistance() + matrix[nodeMap.get(adj.get(looper))][nodeMap.get(curr)] < distances[nodeMap.get(adj.get(looper))]) {
                    distances[nodeMap.get(adj.get(looper))] = tempPath.getDistance() + matrix[nodeMap.get(curr)][nodeMap.get(adj.get(looper))];
                    tempPath.addDistance(matrix[nodeMap.get(curr)][nodeMap.get(adj.get(looper))]);
                    tempPath.getPath().add(adj.get(looper));
                    queue.add(tempPath);
                }
            }

        }
        return data;
    }
}
