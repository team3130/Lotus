package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.*;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private Node head;
    private ArrayList<Node> nodes;
    private Double[][] matrix;

    public Graph() {
        head = new Node(Chassis.getInstance().getPosCV().getX(), Chassis.getInstance().getPosCV().getY());
        nodes.add(head);
    }

    /**
     * A method to clump nodes. This will put the clump in an arraylist of clumps.
     */
    private void ClumpNodes() {
        boolean[] visited = new boolean[nodes.size()];
        Node temp = head;
        visited[0] = true;
        while (!filled(visited)) {
            PriorityQueue<Node> adjacentNodes = adjacent(temp, visited);
            while (!adjacentNodes.isEmpty()) {
                Node temp2 = adjacentNodes.poll();
                putNodeInGraph(temp, temp2, temp.getDistance(temp2));
                temp = temp2;
                visited[nodeMap.get(temp)] = true;
            }
        }
    }

    private boolean filled(boolean[] boolArray) {
        for (boolean item : boolArray) {
            if (!item) {
                return false;
            }
        }
        return true;
    }

    private void putNodeInGraph(Node toBeAdded, Node ConnectedTo, double weight) {
        matrix[nodeMap.get(ConnectedTo)][nodeMap.get(toBeAdded)] = weight;
        matrix[nodeMap.get(toBeAdded)][nodeMap.get(ConnectedTo)] = weight;
    }

    private void addNode(Node newNode) {
        nodes.add(newNode);
        resize();
        ClumpNodes();
    }

    private void resize() {
        matrix = Arrays.copyOf(matrix, nodes.size());
    }

    /**
     * Returns the adjacent nodes in each sector
     * @param start start node
     * @param visited boolean array of visited nodes, index cooresponds to mapped value
     * @return a priority queue of adjacent nodes
     */
    private PriorityQueue<Node> adjacent(Node start, boolean[] visited) {
        // each index corresponds to a sector, starting facing forward and moving 45 degrees to the right at a time
        Node[] shortest = new Node[8];

        // fill the array with the maximum integer value at each spot
        Arrays.fill(shortest, Integer.MAX_VALUE);
        //TODO: add a comparator that sorts the nodes based off of their distance to start
        PriorityQueue<Node> adjacent = new PriorityQueue<>();

        // iterate through nodes
        for (int nodesLooper = 0; nodesLooper < nodes.size(); nodesLooper++) {
                // get the ball closest in the sector and compare it to the previous closer in the sector
            if (shortest[degreesToIndex(nodes.get(nodesLooper).getRelAngle(start))].getDistance(start) < nodes.get(nodesLooper).getDistance(start)) {
                // assign the sector to the new shortest distance node
                shortest[degreesToIndex(nodes.get(nodesLooper).getRelAngle(start))] = nodes.get(nodesLooper);
            }
        }

        for (int shortLooper = 0; shortLooper < shortest.length; shortLooper++) {
            if (shortest[shortLooper].getDistance(start) < Integer.MAX_VALUE && !visited[nodeMap.get(shortest[shortLooper])]) {
                adjacent.add(shortest[shortLooper]);
            }
        }
        return adjacent;
    }

    /**
     * Returns the index of the sector based on the angle
     * @param degrees absolute angle from the object
     * @return the index for the array shortest in {@link #adjacent(Node, boolean[])}
     */
    private int degreesToIndex(double degrees) {
        if (degrees >= 0 && degrees <= 45) {
            return 0;
        }
        else if (degrees > 45 && degrees <= 90) {
            return 1;
        }
        else if (degrees > 90 && degrees <= 135) {
            return 2;
        }
        else if (degrees > 135 && degrees <= 180) {
            return 3;
        }
        else if (degrees > 180 && degrees <= 225) {
            return 4;
        }
        else if (degrees > 225 && degrees <= 270) {
            return 5;
        }
        else if (degrees > 270 && degrees <= 315) {
            return 6;
        }
        else if (degrees > 315 && degrees <= 360) {
            return 7;
        }
        return -1;
    }
}
