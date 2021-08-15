package frc.team3130.robot.SupportingClasses;

import com.sun.jdi.InconsistentDebugInfoException;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

import java.sql.Array;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Bal {
    //TODO: write logic if the ball has been intook
    private boolean intaken = false;

    private double[] positionFix = new double[2];

    private Chassis m_chassis;

    private Supplier<double[]> positionRel;

    public Bal(double[] positionFix, Chassis chassis) {
        this(positionFix[0], positionFix[1], chassis);
    }

    public Bal(double positionFixX, double positionFixY, Chassis chassis) {
        positionFix[0] = positionFixX;
        positionFix[1] = positionFixY;
        m_chassis = chassis;
        positionRel = () -> new double[] {positionFix[0] - m_chassis.getPosCV().getX(), positionFix[1] - m_chassis.getPosCV().getY()};
    }

    public Supplier<double[]> getPositionRel() {
        return positionRel;
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
     * sets position to new position
     * @param temp array that should be of size 2 that holds the new coordinates
     */
    public void updatePositionFix(double[] temp) {
        if (temp.length != 2) {
            System.out.println("Too many or too little values in update position Fix parameter of Bal. num of values: " + temp.length);
        }
        else {
            positionFix[0] = temp[0];
            positionFix[1] = temp[1];
        }
    }

    public double getDistance() {
        double [] temp = positionRel.get();
        return Math.sqrt((temp[0] * temp[0]) + (temp[1] * temp[1]));
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
