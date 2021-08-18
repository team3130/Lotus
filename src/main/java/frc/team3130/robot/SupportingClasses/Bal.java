package frc.team3130.robot.SupportingClasses;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team3130.robot.subsystems.Chassis;

import java.util.function.Supplier;

public class Bal implements Comparable<Bal>{
    //TODO: write logic if the ball has been intook
    private boolean intaken = false;

    private double[] positionRel = new double[2];

    private Chassis m_chassis;

    private Supplier<double[]> positionAbs;

    public Bal(double[] positionRel, Chassis chassis) {
        this(positionRel[0], positionRel[1], chassis);
    }

    public Bal(double positionRelX, double positionRelY, Chassis chassis) {
        positionRel[0] = positionRelX;
        positionRel[1] = positionRelY;
        m_chassis = chassis;
        positionAbs = () -> new double[] {positionRel[0] + m_chassis.getPosCV().getX(), positionRel[1] + m_chassis.getPosCV().getY()};
    }

    public Supplier<double[]> getPositionAbs() {
        return positionAbs;
    }

    public double[] getPositionRel() {
        return positionRel;
    }

    /**
     * sets position to new position
     * @param temp array that should be of size 2 that holds the new coordinates
     */
    public void updatePositionRel(double[] temp) {
        if (temp.length != 2) {
            System.out.println("Too many or too little values in update position Fix parameter of Bal. num of values: " + temp.length);
        }
        else {
            positionRel[0] = temp[0];
            positionRel[1] = temp[1];
        }
    }

    public double getDistance() {
        double [] temp = positionAbs.get();
        return Math.sqrt((temp[0] * temp[0]) + (temp[1] * temp[1]));
    }

    public Pose2d getPose() {
        return new Pose2d(positionRel[0], positionRel[1], new Rotation2d(0));
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Bal temp = ((Bal) o);
        return (temp.positionRel == this.positionRel) || ((temp.positionRel[0] <= this.positionRel[0] + 0.5 && temp.positionRel[0] <= this.positionRel[0] - 0.5) && (temp.positionRel[1] <= this.positionRel[1] + 0.5 && temp.positionRel[1] <= this.positionRel[1] - 0.5));
    }

    /**
     * Overridden "compareTo" to compare objects that the queue will use.
     * The plan for this method is to incorporate more logic to decide whether the ball is in close proximity to other balls\
     * and return a value that incorporates both the distance to the ball, and the distance from the ball to other balls\
     * as well as how many balls we have/need to collect
     * @param bal Object that will be compared to
     * @return an integer value that will be used by the priority heap
     */
    @Override
    public int compareTo(Bal bal) {
        if (this.getDistance() > bal.getDistance()) return 1;
        if (this.equals(bal) || this.getDistance() == bal.getDistance()) return 0;
        else return -1;
    }
}
