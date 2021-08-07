package frc.team3130.robot.SupportingClasses;

import com.sun.jdi.InconsistentDebugInfoException;
import frc.team3130.robot.RobotMap;

public class Bal {
    //TODO: write logic if the ball has been intook
    private boolean intaken = false;

    private double[] positionRel = new double[2], positionFix = new double[2];

    public Bal(double[] positionFix) {
        this(positionFix[0], positionFix[1]);
    }

    public Bal(double positionFixX, double positionFixY) {
        positionFix[0] = positionFixX;
        positionFix[1] = positionFixY;
    }

    public double[] getPositionRel() {
        return positionRel;
    }

    public void setPositionRelX(double positionRelX) {
        positionRel[0] = positionRelX;
    }

    public void setPositionRelY(double positionRelY) {
        positionRel[1] = positionRelY;
    }

    public double[] getPositionFix() {
        return positionFix;
    }

    public void setPositionFixX(double positionFixX) {
        positionFix[0] = positionFixX;
    }

    public void setPositionFixY(double positionFixY) {
        positionFix[1] = positionFixY;
    }

    /**
     * averages current coordinates with new coordinates for the fixed position
     * @param temp array that should be of size 2 that holds the new coordinates
     */
    public void updatePositionFix(double[] temp) {
        if (temp.length > 2) {
            System.out.println("Too many values in update position Fix parameter of Bal");
        }
        else {
            positionFix[0] = (positionFix[0] + temp[0]) / 2;
            positionFix[1] = (positionFix[1] + temp[1]) / 2;
        }
    }

    public double getDistance() {
        return Math.sqrt((positionRel[0] * positionRel[0]) + (positionRel[1] * positionRel[1]));
    }

    @Override
    public boolean equals(Object o) {
        try {
            Bal temp = ((Bal) o);
            return (temp.positionFix == this.positionFix) || ((temp.positionFix[0] <= this.positionFix[0] + 0.5 && temp.positionFix[0] <= this.positionFix[0] - 0.5) && (temp.positionFix[1] <= this.positionFix[1] + 0.5 && temp.positionFix[1] <= this.positionFix[1] - 0.5));
        }
        catch (Exception e) {
            return false;
        }
    }
}
