package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3130.robot.commands.RunWinch;
import frc.team3130.robot.controls.JoystickTrigger;
import frc.team3130.robot.subsystems.Winch;

public class RobotContainer {

    // define Subsystems

    private final Winch m_winch = new Winch();


    //Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);


    // Binding the buttons and triggers that are defined above to respective commands
    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        System.out.println("SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH SOUT BRUH");
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new RunWinch(m_winch, -1)); //R trigger
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new RunWinch(m_winch, 1)); // L trigger also deploys intake while active
    }
}
