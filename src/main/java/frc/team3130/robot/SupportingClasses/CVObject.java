package frc.team3130.robot.SupportingClasses;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class CVObject {

    protected final double x_pos;
    protected final double y_pos;

    protected final Pose2d pos;

    protected PhysicalObject objType;
    protected ObjDimensions dimensions;

    public CVObject(double x_pos, double y_pos, PhysicalObject obj) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        objType = obj;
        pos = new Pose2d(x_pos, y_pos, new Rotation2d(0));
    }
    public double getRelAngle(Node otherNode) {
        return Math.toDegrees(Math.asin(otherNode.getY_pos() - this.getY_pos() / getDistance(otherNode)));
    }
    public double getDistance(Node otherNode) {
        return Math.sqrt(Math.pow(otherNode.getX_pos() - this.getY_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2));
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
}
