package frc.team3130.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team3130.robot.commands.Winch;
import frc.team3130.robot.controls.JoystickTrigger;
import frc.team3130.robot.subsystems.WinchSub;

public class RobotContainer {

    // define Subsystems

    private final WinchSub m_winch = new WinchSub();

    // This section is here IF we need it later
    public WinchSub getWinch() {return m_winch;}

    //Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);


    // Binding the buttons and triggers that are defined above to respective commands
    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new Winch(m_winch, -0.25)); //R trigger
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Winch(m_winch, 5)); // L trigger also deploys intake while active
    }
}

