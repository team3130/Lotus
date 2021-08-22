package frc.team3130.robot.SupportingClasses;

import com.sun.jdi.InconsistentDebugInfoException;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

import java.sql.Array;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Bal {
    //TODO: write logic if the ball has been intook
    private boolean intaken = false;

    private double[] positionFix = new double[2];

    public static Chassis m_chassis;

    private Supplier<double[]> positionRel;

    public Bal(double[] positionFix) {
        this(positionFix[0], positionFix[1]);
    }

    public Bal(double positionFixX, double positionFixY) {
        positionFix[0] = positionFixX;
        positionFix[1] = positionFixY;
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

    public Pose2d getPose() {
        return new Pose2d(positionFix[0], positionFix[1], new Rotation2d(0));
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
