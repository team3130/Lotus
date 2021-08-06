package frc.team3130.robot.SupportingClasses;

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

    public double getDistance() {
        return Math.sqrt((positionRel[0] * positionRel[0]) + (positionRel[1] * positionRel[1]));
    }
}
