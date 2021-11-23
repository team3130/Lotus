package frc.team3130.robot.SupportingClasses;

import java.util.Objects;

public class Node {
    private final double x_pos;
    private final double y_pos;
    private ObjDimensions dimensions;
    private final PhysicalObject typeOfObject;

    public Node(double x_pos, double y_pos, PhysicalObject typeOfObject) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        this.typeOfObject = typeOfObject;

        if (typeOfObject == PhysicalObject.ball) {
            // TODO: find these values
            dimensions = new ObjDimensions(5, 5, 5);
        }
        else if (typeOfObject == PhysicalObject.bot || typeOfObject == PhysicalObject.fieldElement) {
            // TODO: find these values based off of camera
            dimensions = new ObjDimensions(5, 5, 5);
        }
        else {
            //TODO: Find dimensions from rule book after it is released
            dimensions = new ObjDimensions(5, 5, 5);
        }

    }

    public double getX_pos() {
        return x_pos;
    }

    public double getY_pos() {
        return y_pos;
    }

    public ObjDimensions getDimensions() {
        return dimensions;
    }

    public PhysicalObject getTypeOfObject() {
        return typeOfObject;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return Double.compare(node.x_pos, x_pos) == 0 && Double.compare(node.y_pos, y_pos) == 0 && typeOfObject == node.typeOfObject;
    }

    public double getRelAngle(Node otherNode) {
        return Math.toDegrees(Math.asin(otherNode.getY_pos() - this.getY_pos() / getDistance(otherNode)));
    }
    public double getDistance(Node otherNode) {
        if (this.getTypeOfObject() != PhysicalObject.ball || otherNode.getTypeOfObject() != PhysicalObject.ball) {
            return Double.MAX_VALUE;
        }
        return Math.sqrt(Math.pow(otherNode.getX_pos() - this.getY_pos(), 2) + Math.pow(otherNode.getY_pos() - this.getY_pos(), 2));
    }
}
