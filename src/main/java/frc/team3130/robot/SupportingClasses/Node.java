package frc.team3130.robot.SupportingClasses;

public class Node {
    private final double x_pos;
    private final double y_pos;
    private final Pose2d pos;

    public Node(double x_pos, double y_pos) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        pos = new Pose2d(x_pos, y_pos, new Rotation2d(0));
    }

    public Pose2d getPos() {
        return pos;
    }

    public double getX_pos() {
        return x_pos;
    }

    public double getY_pos() {
        return y_pos;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        // if its within 0.5 units
        if (this.x_pos < o.x_pos + 0.5 && this.x_pos > o.x_pos - 0.5 && this.y_pos < o.y_pos + 0.5 && this.y_pos > o.y_pos - 0.5) {return true;}
    }

    public double getRelAngle(Node otherNode) {
        return Math.toDegrees(Math.asin(otherNode.getY_pos() - this.getY_pos() / getDistance(otherNode)));
    }

    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.pow(otherNode.getX_pos() - this.getX_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2));
    }
}
