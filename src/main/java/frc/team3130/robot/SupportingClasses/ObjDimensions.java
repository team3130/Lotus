package frc.team3130.robot.SupportingClasses;

public class ObjDimensions {
    // all of the measurements below are in meters
    private final double length;
    private final double width;
    private final double height;

    /**
     * Constructor for ObjDimensions
     * Stores the rectangular directions of an object
     * @param length in m
     * @param width in m
     * @param height in m
     */
    public ObjDimensions(double length, double width, double height) {
        this.length = length;
        this.width = width;
        this.height = height;
    }

    /**
     * gets length
     * @return length in meters
     */
    public double getLength() {
        return length;
    }

    /**
     * getter for width
     * @return width in meters
     */
    public double getWidth() {
        return width;
    }

    /**
     * getter for height
     * @return height in meters
     */
    public double getHeight() {
        return height;
    }
}
