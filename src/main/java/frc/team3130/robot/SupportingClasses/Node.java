package frc.team3130.robot.SupportingClasses;

public class Node extends CVObject{

    public Node(double x_pos, double y_pos) {
        super(x_pos, y_pos, PhysicalObject.ball);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return Double.compare(node.x_pos, x_pos) == 0 && Double.compare(node.y_pos, y_pos) == 0;
    }

}
