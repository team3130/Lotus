package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3130.robot.commands.Hopper.HopperIn;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.WheelOfFortune.ColorAlignment;
import frc.team3130.robot.commands.WheelOfFortune.TestHSB;
import frc.team3130.robot.commands.WheelOfFortune.TripleSpinFinish;


public class OI {
    private class JoystickTrigger extends Trigger {

        private Joystick stick;
        private int axis;
        private double threshold;

        private JoystickTrigger(Joystick stick, int axis) {
            this.stick = stick;
            this.axis = axis;
            threshold = 0.1;
        }

        private JoystickTrigger(Joystick stick, int axis, double threshold) {
            this.stick = stick;
            this.axis = axis;
            this.threshold = threshold;
        }

        @Override
        public boolean get() {
            return stick.getRawAxis(axis) > threshold;
        }

    }

    private class POVTrigger extends Trigger {

        private Joystick stick;
        private int POV;

        public POVTrigger(Joystick stick, int POV) {
            this.stick = stick;
            this.POV = POV;
        }

        @Override
        public boolean get() {
            return stick.getPOV(0) == POV;
        }

    }

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
    public static Joystick driverGamepad;
    public static Joystick weaponsGamepad;

    /**
     * Definitions for joystick buttons start
     */
    private static JoystickButton spinWheel;
    private static JoystickButton spinShooter;

    private static JoystickButton testColorAlignment;

    private static JoystickButton testTripleSpinFinish;

    private static JoystickTrigger intakeIn;
    private static JoystickButton intakeOut;
    private static JoystickTrigger hopperIn;
    private static JoystickButton hopperOut;

    private static JoystickButton testTestHSB;

    public void checkTriggers() {
        //Driver
        if (Math.abs(OI.driverGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER)) >= RobotMap.kIntakeTriggerDeadband) {

        } else {

        }
        if (Math.abs(OI.driverGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER)) >= RobotMap.kIntakeTriggerDeadband) {

        } else {

        }
    }

    //Settings for gamepad
    private OI() {
        driverGamepad = new Joystick(0);
        weaponsGamepad = new Joystick(1);

        //spinWheel = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A);

        testTestHSB = new JoystickButton(driverGamepad, RobotMap.LST_BTN_A);

        spinShooter = new JoystickButton(driverGamepad, RobotMap.LST_BTN_X);

        testTripleSpinFinish = new JoystickButton(driverGamepad, RobotMap.LST_BTN_B);

        testColorAlignment = new JoystickButton(driverGamepad, RobotMap.LST_BTN_Y);


        intakeIn = new JoystickTrigger(driverGamepad, RobotMap.LST_BTN_RBUMPER);
        intakeOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_LBUMPER);
        hopperIn = new JoystickTrigger(driverGamepad, RobotMap.LST_AXS_RTRIGGER);
        hopperOut = new JoystickButton(driverGamepad, RobotMap.LST_BTN_RBUMPER);



        intakeIn.whenActive(new IntakeIn());
        intakeOut.whileHeld(new IntakeOut());
        hopperIn.whenActive(new HopperIn());
        hopperOut.whileHeld(new HopperOut());

        //spinWheel.whileHeld(new SpinWheel(spinWheel));

        //spinWheel.whileHeld(new SpinShooter());

        testTripleSpinFinish.whenPressed(new TripleSpinFinish());

        testColorAlignment.whenPressed(new ColorAlignment());

        testTestHSB.whenActive(new TestHSB());





    }
}

