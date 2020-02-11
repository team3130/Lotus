package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Hood implements Subsystem {

    //Create necessary objects
    private static Solenoid m_hoodPistons;


    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Hood. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Hood INSTANCE = new Hood();

    /**
     * Returns the Singleton instance of this Hood. This static method
     * should be used -- {@code Hood.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Hood getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Hood.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Hood() {
        m_hoodPistons = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_HOODPISTONS);
        m_hoodPistons.set(false);
    }

    public void toggleHoodPistons() {
        if (m_hoodPistons.get() == false) {
            m_hoodPistons.set(true);
        } else {
            m_hoodPistons.set(false);
        }
    }

}

