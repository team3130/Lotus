package frc.team3130.robot.SupportingClasses;

import junit.framework.TestCase;

public class GraphTest extends TestCase {
    public void testGetPath() {
        double[][] testArr = {{1, 3.5}, {1, 4}, {1, 5}, {2.2, 4.5}, {3, 4}, {3.5, 4}, {4.5, 5}};
        for (double[] doubles : testArr) {
            Graph.getInstance().addNode(new Node(doubles[0], doubles[1]));
        }

        Graph.getInstance().printGraph();

        assertFalse(Graph.getInstance().containsDuplicates());
        assertFalse(Graph.getInstance().containsNan());
    }
}