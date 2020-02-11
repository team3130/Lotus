package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExampleSubsystem implements Subsystem {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this ExampleSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static ExampleSubsystem INSTANCE = new ExampleSubsystem();

    /**
     * Returns the Singleton instance of this ExampleSubsystem. This static method
     * should be used -- {@code ExampleSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static ExampleSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ExampleSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private ExampleSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }

}

