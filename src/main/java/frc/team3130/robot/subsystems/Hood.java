package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Hood implements Subsystem {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private static Solenoid m_hoodPistons;


    //Create and define all standard data types needed

    /**
     * The Singleton instance of this ExampleSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Hood INSTANCE = new Hood();

    /**
     * Creates a new instance of this ExampleSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Hood() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.

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


    /**
     * Returns the Singleton instance of this ExampleSubsystem. This static method
     * should be used -- {@code ExampleSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Hood getInstance() {
        return INSTANCE;
    }

}

