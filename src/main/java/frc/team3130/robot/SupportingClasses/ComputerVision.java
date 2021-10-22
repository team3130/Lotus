package frc.team3130.robot.SupportingClasses;

/**
 * <p>An abstract class for computer vision classes</p>
 * <p>
 *     Requires the instantiation of a rotation matrix and a translation vector
 *     has a predict method
 * </p>
 */
public abstract class ComputerVision {
    protected double[][] rotation;
    protected double[] translation;

    /**
     * <p>predicts the position of the ball relative to the location of the bot</p>
     * @param pixel pixel value given by camera
     * @return a matrix of the relative position of the ball
     */
    protected double[] predict(double[] pixel) {
        double[] coords = new double[2];
        coords[0] = ((pixel[0] * rotation[0][0]) + (pixel[1] * rotation[0][1])) + translation[0];
        coords[1] = ((pixel[0] * rotation[1][0]) + (pixel[1] * rotation[1][1])) + translation[1];
        return coords;
    }
}
