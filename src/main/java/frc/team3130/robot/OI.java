package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.DeployBigClimber;
import frc.team3130.robot.commands.Climber.DeploySmallClimber;
import frc.team3130.robot.commands.Hood.MoveHood;
import frc.team3130.robot.commands.Hood.ToggleHood;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Shoot;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.ShootNear;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.commands.WheelOfFortune.ColorAlignment;
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
//    private static JoystickButton testFlywheel = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER); //R bumper
    private static JoystickButton shoot = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER); //R bumper
    private static JoystickButton hopperOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LBUMPER); //L bumper
    private static JoystickButton aimTurret = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS); //R joystick press
    private static JoystickButton toggleIntake = new JoystickButton(driverGamepad, RobotMap.LST_BTN_MENU); //Menu button
    private static JoystickButton shift = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS); //L joystick press
    private static JoystickButton nearShoot = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A); //Button A


    /**
     * Weapons
     */

//    private static POVButton incrementShooterOffset = new POVButton(weaponsGamepad, RobotMap.LST_POV_N);
//    private static POVButton decrementShooterOffset = new POVButton(weaponsGamepad, RobotMap.LST_POV_S);
//    private static POVButton resetShooterOffset = new POVButton(weaponsGamepad, RobotMap.LST_POV_E);

    private static POVButton hoodUp = new POVButton(driverGamepad, RobotMap.LST_POV_N);
    private static POVButton hoodDown = new POVButton(driverGamepad, RobotMap.LST_POV_S);

    private static JoystickButton toggleClimber = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_MENU); //Menu button
    private static JoystickButton retractClimber = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_WINDOW); //Windows button
    private static JoystickButton toggleWOF = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_A);
    private static JoystickButton wofLeft = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_X);
    private static JoystickButton wofRight = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_B);
    private static JoystickButton testTripleSpinFinish = new JoystickButton(weaponsGamepad, RobotMap.LST_BTN_Y);


    // Binding the buttons and triggers that are defined above to respective commands
    private OI() {
        intakeIn.whenHeld(new IntakeIn());
        intakeOut.whenHeld(new IntakeOut());

        shoot.whenHeld(new Shoot());
        hopperOut.whenHeld(new HopperOut());

//        testColorAlignment.whenPressed(new ColorAlignment());
//        testTestHSB.whenHeld(new TestHSB());

        aimTurret.whenPressed(new ToggleTurretAim());
//        testFlywheel.whenHeld(new TuneFlywheelRPM());

//        incrementShooterOffset.whenPressed(new IncrementRPM());
//        decrementShooterOffset.whenPressed(new DecrementRPM());
//        resetShooterOffset.whenPressed(new ResetRPM());

        hoodUp.whenHeld(new MoveHood(1));
        hoodDown.whenHeld(new MoveHood(-1));

        toggleIntake.whenPressed(new ToggleIntake());

        toggleClimber.whenPressed(new DeploySmallClimber());
        retractClimber.whenPressed(new DeployBigClimber());

        toggleWOF.whenPressed(new ToggleWOF());
        testTripleSpinFinish.whenPressed(new ColorAlignment());
        wofLeft.whenHeld(new SpinWOFLeft());
        wofRight.whenHeld(new SpinWOFRight());

        shift.whenPressed(new ShiftToggle());

        nearShoot.whenHeld(new ShootNear());
    }
}

