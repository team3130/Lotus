package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Function;

public class Graph {
    HashMap<Node, Integer> nodeMap;
    private Node head;
    private ArrayList<Node> nodes;
    private ArrayList<ArrayList<Node>> Clumps;

    public Graph() {
        head = new Node(Chassis.getInstance().getPosCV().getX(), Chassis.getInstance().getPosCV().getY(), PhysicalObject.bot);
        nodes.add(head);

    }

    /**
     * A method to clump nodes. This will put the clump in an arraylist of clumps.
     */
    private void ClumpNodes(Node start) {
        // each index corresponds to a sector, starting facing forward and moving 45 degrees to the right at a time
        double[] shortest = new double[8];

        // fill the array with the maximum integer value at each spot
        Arrays.fill(shortest, Integer.MAX_VALUE);

        // iterate through nodes
        for (int nodesLooper = 0; nodesLooper < nodes.size(); nodesLooper++) {
            // make sure that we are checking a ball
            if (nodes.get(nodesLooper).getTypeOfObject() == PhysicalObject.ball) {
                // get the ball closest in the sector and compare it to the previous closer in the sector
                if (shortest[degressToIndex(nodes.get(nodesLooper).getRelAngle(start))] < nodes.get(nodesLooper).getDistance(start)) {
                    // assign the sector to the new shortest distance
                    shortest[degressToIndex(nodes.get(nodesLooper).getRelAngle(start))] = nodes.get(nodesLooper).getDistance(start);
                }
            }
        }
    }

    private int degressToIndex(double degrees) {
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
