package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.security.spec.ECField;

public class ExampleSubsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private static final ExampleSubsystem instance = new ExampleSubsystem();

    /**
     * <p>Instance handling for the subsystem</p>
     * @return the object as a singleton
     */
    public static ExampleSubsystem getInstance() {
        return instance;
    }

    //Create and define all standard data types needed

    /**
     * <p>Constructor for the subsystem</p>
     */
    private ExampleSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }

}

