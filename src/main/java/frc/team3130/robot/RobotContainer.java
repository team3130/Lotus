package frc.team3130.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team3130.robot.Auton.Chooser;
import frc.team3130.robot.commands.Chassis.DefaultDrive;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.DeployBigClimber;
import frc.team3130.robot.commands.Climber.DeploySmallClimber;
import frc.team3130.robot.commands.Hood.MoveHood;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Shoot.Shoot;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.Shoot.ShootNear;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.commands.WheelOfFortune.ColorAlignment;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFLeft;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFRight;
import frc.team3130.robot.commands.WheelOfFortune.ToggleWOF;
import frc.team3130.robot.controls.JoystickTrigger;
import frc.team3130.robot.subsystems.*;


import java.util.*;

public class RobotContainer {
    //see here for references if lost: https://github.com/wpilibsuite/allwpilib/blob/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/RobotContainer.java

    // define Subsystems
    private final Chassis m_chassis = new Chassis();
    private final Climber m_climber = new Climber();
    private final Flywheel m_flyWheel = new Flywheel();
    private final Hood m_hood = new Hood();
    private final Hopper m_hoppper = new Hopper();
    private final Intake m_intake = new Intake();
    private final Turret m_turret = new Turret();
    private final WheelOfFortune m_wheelOfFortune = new WheelOfFortune();
    private final Chooser m_chooser = new Chooser(m_chassis);

    // This section is here IF we need it later
    public Chassis getChassis() {return m_chassis;}
    public Climber getClimber() {return m_climber;}
    public Flywheel getFlywheel() {return m_flyWheel;}
    public Hood getHood() {return m_hood;}
    public Hopper getHopper() {return m_hoppper;}
    public Intake getIntake() {return m_intake;}
    public Turret getTurret() {return m_turret;}
    public WheelOfFortune getWOF() {return m_wheelOfFortune;}
    public Chooser getChooser() {return m_chooser;}


    //Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    // private final Command m_BarrelRacing = new BarrelRacing(60, m_chassis);


    // Binding the buttons and triggers that are defined above to respective commands
    public RobotContainer() {
        m_chooser.chooserRegistry();
        configureButtonBindings();
        m_chassis.setDefaultCommand(
                new DefaultDrive(
                        m_chassis,
                        () -> m_driverGamepad.getY(GenericHID.Hand.kLeft),
                        () -> m_driverGamepad.getX(GenericHID.Hand.kRight)
                )
        );

        /*
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
        m_chooser.addOption("Complex Auto", m_complexAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);
         */

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
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenPressed(new ToggleIntake(m_intake));; //Menu button
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new ShiftToggle(m_chassis)); //L joystick press
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new ShootNear(m_turret, m_hoppper, m_flyWheel, m_hood)); //Button A


        /*
         * Weapons
         */

        new POVButton(m_driverGamepad, RobotMap.LST_POV_N).whenHeld(new MoveHood(1, m_hood));
        new POVButton(m_driverGamepad, RobotMap.LST_POV_S).whenHeld(new MoveHood(-1, m_hood));

        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeploySmallClimber(m_climber));; //Menu button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_WINDOW).whenPressed(new DeployBigClimber(m_climber));; //Windows button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenPressed(new ToggleWOF(m_wheelOfFortune, m_intake));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new SpinWOFLeft(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SpinWOFRight(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenPressed(new ColorAlignment(m_wheelOfFortune));


    }

    public void reset(){
        m_chassis.reset();
    }

}

