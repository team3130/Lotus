package frc.team3130.robot.SupportingClasses;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private Node head;
    private ArrayList<Node> nodes;

    private double determineWeight(Node nodeOne, Node nodeTwo) {
        if (nodeOne.getTypeOfObject() != PhysicalObject.ball || nodeTwo.getTypeOfObject() != PhysicalObject.ball) {
            return Double.MAX_VALUE;
        }
        //TODO: change to incoporate more factors than Euclidean distance such as proximity to other balls and possible paths (a lot of math and bounding boxes required for this)
        double weight = Math.sqrt(Math.pow(nodeTwo.getX_pos() - nodeOne.getY_pos(), 2) + Math.pow(nodeTwo.getY_pos() - nodeOne.getY_pos(), 2));
        
        return weight;
    }
}
