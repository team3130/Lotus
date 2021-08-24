package frc.team3130.robot.SupportingClasses;

public abstract class ComputerVision {
    protected double[][] rotation;
    protected double[] translation;

    protected double[] predict(double[] pixel) {
        double[] coords = new double[2];
        coords[0] = ((pixel[0] * rotation[0][0]) + (pixel[1] * rotation[0][1])) + translation[0];
        coords[1] = ((pixel[0] * rotation[1][0]) + (pixel[1] * rotation[1][1])) + translation[1];
        return coords;
    }
}
