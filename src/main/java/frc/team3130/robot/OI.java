package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.ClimberPull;
import frc.team3130.robot.commands.Climber.ClimberUnpull;
import frc.team3130.robot.commands.Climber.ToggleClimber;
import frc.team3130.robot.commands.Flywheel.SetFlywheelRPM;
import frc.team3130.robot.commands.Hood.ActuateHood;
import frc.team3130.robot.commands.Hopper.HopperIn;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.controls.JoystickTrigger;

public class OI {


    //Instance Handling
    private static OI m_pInstance;

    public static OI GetInstance() {
        if (m_pInstance == null) m_pInstance = new OI();
        return m_pInstance;
    }


    public static double getSkywalker() {
        double spin = 0;
        spin += driverGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER);
        spin -= driverGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER);
        return spin;
    }

    //Joysticks
    public static Joystick driverGamepad = new Joystick(0);
    public static Joystick weaponsGamepad = new Joystick(1);

    /**
     * Definitions for joystick buttons start
     */
    private static JoystickButton testTurret = new JoystickButton(driverGamepad, RobotMap.LST_BTN_B);
    private static JoystickTrigger testFlywheel = new JoystickTrigger(driverGamepad, RobotMap.LST_AXS_RTRIGGER);

//    private static JoystickButton testColorAlignment = new JoystickButton(driverGamepad, RobotMap.LST_BTN_Y);
//    private static JoystickButton testTripleSpinFinish = new JoystickButton(driverGamepad, RobotMap.LST_BTN_B);
    private static JoystickButton intakeOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER);
    private static JoystickButton intakeIn = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LBUMPER);
    private static JoystickButton hopperIn = new JoystickButton(driverGamepad, RobotMap.LST_BTN_X);
    private static JoystickButton hopperOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A);


//    private static JoystickButton testTestHSB = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A);
    private static JoystickButton toggleIntake = new JoystickButton(driverGamepad, RobotMap.LST_BTN_WINDOW);
    private static JoystickButton shift = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS);
    private static JoystickButton toggleHood = new JoystickButton(driverGamepad, RobotMap.LST_BTN_MENU);
    private static JoystickButton toggleClimber = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_MENU);
    private static JoystickButton climberPull = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_RBUMPER);
    private static JoystickButton climberunpull = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_LBUMPER);


    // Binding the buttons and triggers that are defined above to respective commands
    private OI() {
        intakeIn.whenHeld(new IntakeIn());
        intakeOut.whenHeld(new IntakeOut());

        hopperIn.whenHeld(new HopperIn());
        hopperOut.whenHeld(new HopperOut());

//        testTripleSpinFinish.whenPressed(new TripleSpinFinish());
//        testColorAlignment.whenPressed(new ColorAlignment());
//        testTestHSB.whenHeld(new TestHSB());

        testTurret.whenPressed(new ToggleTurretAim());
        testFlywheel.whenHeld(new SetFlywheelRPM());

        toggleIntake.whenPressed(new ToggleIntake());

        toggleClimber.whenPressed(new ToggleClimber());
        climberPull.whileHeld(new ClimberPull());
        climberunpull.whileHeld(new ClimberUnpull());

        shift.whenPressed(new ShiftToggle());

        toggleHood.whenPressed(new ActuateHood());
    }
}

