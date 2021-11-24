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

    private void addNode(Node newNode) {
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
}
