package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.*;
import frc.team3130.robot.commands.Flywheel.SetFlywheelRPM;
import frc.team3130.robot.commands.Hood.ActuateHood;
import frc.team3130.robot.commands.Hopper.HopperIn;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFLeft;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFRight;
import frc.team3130.robot.commands.WheelOfFortune.ToggleWOF;
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

    /**
     * Drivers
     */
    private static JoystickTrigger intakeOut = new JoystickTrigger(driverGamepad, RobotMap.LST_AXS_RTRIGGER); //R trigger
    private static JoystickTrigger intakeIn = new JoystickTrigger(driverGamepad, RobotMap.LST_AXS_LTRIGGER); // L trigger also deploys intake while active

//    private static JoystickButton testColorAlignment = new JoystickButton(driverGamepad, RobotMap.LST_BTN_Y);
//    private static JoystickButton testTripleSpinFinish = new JoystickButton(driverGamepad, RobotMap.LST_BTN_B);
    private static JoystickButton testFlywheel = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER); //R bumper
    private static JoystickButton hopperIn = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER ); //R bumper TODO: check
    private static JoystickButton hopperOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LBUMPER); //L bumper
    private static JoystickButton testTurret = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS); //R joystick press
    private static JoystickButton toggleIntake = new JoystickButton(driverGamepad, RobotMap.LST_BTN_MENU); //Menu button
    private static JoystickButton shift = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS); //L joystick press
    private static JoystickButton toggleHood = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A); //Button A

//    private static JoystickButton testTestHSB = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A);

    /**
    * Weapons
     */
    private static JoystickTrigger RightWinchUnpull = new JoystickTrigger(weaponsGamepad, RobotMap.LST_AXS_RTRIGGER);
    private static JoystickTrigger LeftWinchUnpull = new JoystickTrigger(weaponsGamepad, RobotMap.LST_AXS_LTRIGGER);

    private static JoystickButton toggleClimber = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_MENU); //Menu button
    private static JoystickButton RightWinchPull = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_RBUMPER);
    private static JoystickButton LeftWinchPull = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_LBUMPER);
    private static JoystickButton toggleWOF = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_A);
    private static JoystickButton WOFLeft = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_X);
    private static JoystickButton WOFRight = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_B);




    // Binding the buttons and triggers that are defined above to respective commands
    private OI() {
        intakeIn.whenHeld(new IntakeIn());
        intakeOut.whenHeld(new IntakeOut());

        hopperIn.whenHeld(new HopperIn());
        hopperOut.whenHeld(new HopperOut());

//        testTripleSpinFinish.whenPressed(new TripleSpinFinish());
//        testColorAlignment.whenPressed(new ColorAlignment());
//        testTestHSB.whenHeld(new TestHSB());

        testTurret.toggleWhenActive(new ToggleTurretAim()); // TODO: check this
        testFlywheel.whenHeld(new SetFlywheelRPM());

        toggleIntake.whenPressed(new ToggleIntake());

        toggleClimber.whenPressed(new ToggleClimber());
        RightWinchPull.whenHeld(new RightClimberPull());
        LeftWinchPull.whenHeld(new LeftClimberPull());
        RightWinchUnpull.whenHeld(new RightClimberUnpull());
        LeftWinchUnpull.whenHeld(new LeftClimberUnpull());

        toggleWOF.whenPressed(new ToggleWOF());
        WOFLeft.whenHeld(new SpinWOFLeft());
        WOFRight.whenHeld(new SpinWOFRight());

        shift.whenPressed(new ShiftToggle());

        toggleHood.whenPressed(new ActuateHood());
    }
}

