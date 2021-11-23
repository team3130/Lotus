package frc.team3130.robot.SupportingClasses;

public class Node {
    private double x_pos;
    private double y_pos;
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

    public void setX_pos(double x_pos) {
        this.x_pos = x_pos;
    }

    public double getY_pos() {
        return y_pos;
    }

    public void setY_pos(double y_pos) {
        this.y_pos = y_pos;
    }

    public ObjDimensions getDimensions() {
        return dimensions;
    }

    public PhysicalObject getTypeOfObject() {
        return typeOfObject;
    }
}
