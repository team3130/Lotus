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
        // if its within 0.5 units
        return this.x_pos < node.x_pos + 0.5 && this.x_pos > node.x_pos - 0.5 && this.y_pos < node.y_pos + 0.5 && this.y_pos > node.y_pos - 0.5;
    }

    public double getRelAngle(Node otherNode) {
        return Math.toDegrees(Math.asin(otherNode.getY_pos() - this.getY_pos() / getDistance(otherNode)));
    }

    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.pow(otherNode.getX_pos() - this.getX_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2));
    }
}
