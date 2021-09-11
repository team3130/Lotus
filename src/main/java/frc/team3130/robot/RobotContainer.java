package frc.team3130.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3130.robot.commands.Chassis.DefaultDrive;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.DeployBigClimber;
import frc.team3130.robot.commands.Climber.DeploySmallClimber;
import frc.team3130.robot.commands.Flywheel.SetFlywheelRPM;
import frc.team3130.robot.commands.Hopper.HopperOut;

import frc.team3130.robot.commands.Hopper.Spindexer;
import frc.team3130.robot.commands.Shoot.Shoot;

import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.Turret.ManualTurretAim;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.commands.WheelOfFortune.ColorAlignment;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFLeft;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFRight;
import frc.team3130.robot.commands.WheelOfFortune.ToggleWOF;
import frc.team3130.robot.controls.JoystickTrigger;
import frc.team3130.robot.subsystems.*;

public class RobotContainer {

    // define Subsystems
    private final Chassis m_chassis = new Chassis();
    private final Climber m_climber = new Climber();
    private final Flywheel m_flyWheel = new Flywheel();
    private final Hood m_hood = new Hood();
    private final Hopper m_hoppper = new Hopper();
    private final Intake m_intake = new Intake();
    private final Turret m_turret = new Turret();
    private final WheelOfFortune m_wheelOfFortune = new WheelOfFortune();
    private final Hood m_pnHood = new Hood();

    // This section is here IF we need it later (Update: we needed it)
    public Chassis getChassis() {return m_chassis;}
    public Climber getClimber() {return m_climber;}
    public Flywheel getFlywheel() {return m_flyWheel;}
    public Hood getHood() {return m_hood;}
    public Hopper getHopper() {return m_hoppper;}
    public Intake getIntake() {return m_intake;}
    public Turret getTurret() {return m_turret;}
    public WheelOfFortune getWOF() {return m_wheelOfFortune;}


    public static double getSkywalker() {
        double spin = 0;
        spin += m_driverGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER);
        spin -= m_driverGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER);
        return spin;
    }

    //Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);


    // Binding the buttons and triggers that are defined above to respective commands
    public RobotContainer() {
        configureButtonBindings();
        m_chassis.setDefaultCommand(
                new DefaultDrive(
                        m_chassis,
                        () -> m_driverGamepad.getY(GenericHID.Hand.kLeft),
                        () -> m_driverGamepad.getX(GenericHID.Hand.kRight)
                )
        );
        m_turret.setDefaultCommand(
                new ManualTurretAim(
                        m_turret
                )
        );

    }

    private void configureButtonBindings() {

        /*
         * Definitions for joystick buttons start
         */

        /*
         * Drivers
         */
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_AXS_RTRIGGER).whenHeld(new IntakeIn(m_intake)); //R trigger
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new IntakeOut(m_intake)); // L trigger also deploys intake while active

        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Shoot(m_turret, m_hoppper, m_flyWheel, m_hood)); //R bumper
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new HopperOut(m_hoppper)); //L bumper
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS).whenPressed(new ToggleTurretAim(m_turret)); //R joystick press
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenPressed(new ToggleIntake(m_intake)); //Menu button
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new ShiftToggle(m_chassis)); //L joystick press
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_X).whenHeld(new SetFlywheelRPM(m_flyWheel, m_hoppper));
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new Spindexer(m_hoppper));

        /*

         * Weapons
         */



        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeploySmallClimber(m_climber)); //Menu button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_WINDOW).whenPressed(new DeployBigClimber(m_climber)); //Windows button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenPressed(new ToggleWOF(m_wheelOfFortune, m_intake));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new SpinWOFLeft(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SpinWOFRight(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenPressed(new ColorAlignment(m_wheelOfFortune));

    }

/*    private void setDefaultCommand() {
        *//*
        //TODO: fix this I have no fricking clue what is going on here
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis, () -> driverGamepad.getY(GenericHID.Hand.kLeft), () -> driverGamepad.getX(GenericHID.Hand.kRight)));
        m_climber.setDefaultCommand(new Climber(m_climber, () -> driverGamepad.));
        m_turret.setDefaultCommand(//I DONT KNOW WHATS GOIN OOOOONNNNNNNNNNNNNNNN SETTING DEFAULT COMMANDS ARE WWWWWEEEEEIIIIRRRRDDDD );
        TODO: get Caleb some xanax
         *//*
    }*/
}

