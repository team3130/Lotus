package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private Node head;
    private ArrayList<Node> nodes;
    private ArrayList<ArrayList<Node>> Clumps;

    private void ClumpNodes() {
        for (int i = 0; i < )
    }

    private double getDistance(Node nodeOne, Node nodeTwo) {
        if (nodeOne.getTypeOfObject() != PhysicalObject.ball || nodeTwo.getTypeOfObject() != PhysicalObject.ball) {
            return Double.MAX_VALUE;
        }
        return Math.sqrt(Math.pow(nodeTwo.getX_pos() - nodeOne.getY_pos(), 2) + Math.pow(nodeTwo.getY_pos() - nodeOne.getY_pos(), 2));
    }
}
