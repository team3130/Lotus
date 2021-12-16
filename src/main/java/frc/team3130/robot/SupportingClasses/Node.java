package frc.team3130.robot.SupportingClasses;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Node {
    protected final double x_pos;
    protected final double y_pos;

    protected final Pose2d pos;

    public Node(double x_pos, double y_pos) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        pos = new Pose2d(x_pos, y_pos, new Rotation2d(0));
    }
    public double getX_pos() {
        return x_pos;
    }

    public double getY_pos() {
        return y_pos;
    }

    public Pose2d getPos() {
        return pos;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        // if its within 0.5 units
        return this.x_pos < node.x_pos + 0.1 && this.x_pos > node.x_pos - 0.1 && this.y_pos < node.y_pos + 0.1 && this.y_pos > node.y_pos - 0.1;
    }

    public double getRelAngle(Node otherNode) {
        return Math.toDegrees(Math.asin(otherNode.getY_pos() - this.getY_pos() / getDistance(otherNode)));
    }

    /**
     * Using the law of cosines this method gets the angle to one node if aproached from another
     * @param to node going to
     * @param from node coming to this node from
     * @return the angle measure
     */
    public double getAngleToFrom(Node to, Node from) {
        // Side a in a triangle
        double A = from.getDistance(this);
        // Side b in a triangle
        double B = this.getDistance(to);
        // Side c in a triangle
        double C = to.getDistance(from);
        // set the heading using law of cosines
        return 180 - Math.acos((Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B));
    }

    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.pow(otherNode.getX_pos() - this.getX_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2));
    }
}
