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
        // if its within 0.1 units, it's the same ball
        return this.x_pos < node.x_pos + 0.1 && this.x_pos > node.x_pos - 0.1 && this.y_pos < node.y_pos + 0.1 && this.y_pos > node.y_pos - 0.1;
    }

    public double getRelAngle(Node otherNode) {
        int offset = 0;
        int sign = 1;
        if (otherNode.x_pos - this.x_pos < 0 && otherNode.y_pos - this.y_pos > 0) {
            offset = 90;
        }

        else if (otherNode.x_pos - this.x_pos < 0 && otherNode.y_pos - this.y_pos < 0) {
            offset = 90;
            sign = -1;
        }

        else if (otherNode.x_pos - this.x_pos > 0 && otherNode.y_pos - this.y_pos < 0) {
            sign = -1;
        }

        double degrees = sign * (Math.toDegrees(Math.acos(Math.abs((otherNode.x_pos - this.x_pos) / getDistance(otherNode)))) + offset);

        if (Double.isNaN(degrees)) {
            System.out.println("\n\nArc cosine: " + Math.acos(Math.abs((otherNode.x_pos - this.x_pos) / getDistance(otherNode))));
            System.out.println("Passed into arc cosine: " + Math.abs((otherNode.x_pos - this.x_pos) / getDistance(otherNode)));
            System.out.println("x - x: " + (otherNode.x_pos - this.x_pos));
            System.out.println("Distance: " + getDistance(otherNode));
            System.out.println("offset: " + offset);
            System.out.println("sign: " + sign);
            System.out.println("\n");
            return 0;
        }
        return degrees;
    }

    /**
     * Using the law of cosines this method gets the angle to one node if approached from another
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

        // check if A or B or C is 0 which would result in NaN
        if (A == 0 || B == 0 || C == 0) {
            return 0;
        }

        // set the heading using law of cosines
        double degrees = 180 - Math.toDegrees(Math.acos((Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B)));

        // logic for different quadrants
        int offset = 0;
        int sign = 1;
        if (to.x_pos - this.x_pos < 0 && to.y_pos - this.y_pos > 0) {
            offset = 90;
        }

        else if (to.x_pos - this.x_pos < 0 && to.y_pos - this.y_pos < 0) {
            offset = 90;
            sign = -1;
        }

        else if (to.x_pos - this.x_pos > 0 && to.y_pos - this.y_pos < 0) {
            sign = -1;
        }

        // applying offsets
        degrees = sign * (degrees + offset);

        // debugging NaN
        if (Double.isNaN(degrees)) {
            System.out.println("\n\nArc cosine: " + Math.acos((Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B)));
            System.out.println("Passed into arc cosine: " + (Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(C, 2)) / (2 * A * B));
            System.out.println("A: " + A);
            System.out.println("B: " + B);
            System.out.println("C: " + C);
            System.out.println("\n");
            return 0;
        }
        return degrees;
    }

    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.abs(Math.pow(otherNode.getX_pos() - this.getX_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2)));
    }

    public String toString() {
        return "(" + x_pos + ", " + y_pos + ")";
    }
}
