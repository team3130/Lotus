package frc.team3130.robot.SupportingClasses;

import junit.framework.TestCase;

public class GraphTest extends TestCase {
    public void testGetPath() {
        double[][] testArr = {{1, 3.5}, {1, 4}, {3, 5}, {2.2, 4.5}, {3, 4}, {3.5, 4}, {6.5, 9}};
        for (double[] doubles : testArr) {
            Graph.getInstance().addNode(new Node(doubles[0], doubles[1]));
        }

        // Graph.getInstance().printGraph();

        assertFalse(Graph.getInstance().containsDuplicates());
        assertFalse(Graph.getInstance().containsNan());

        // asserts that the size is 5
        GraphPath bruteForce = Graph.getInstance().bruteForce(5);
        System.out.println(bruteForce);
        assertEquals(5, bruteForce.getPath().size());

        System.out.println("\n\n\n" + Graph.getInstance().getPath(5));

    }
}