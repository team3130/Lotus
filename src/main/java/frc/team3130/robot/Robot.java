// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3130.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3130.robot.IntakeCommand.IntakeIn;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_autonomousCommandTwo;

  private RobotContainer m_robotContainer;

  private double m_autonCommandOneTimer;

  private boolean m_timerCheck;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommandTwo = m_robotContainer.getSecondAutonomousCommand();
    m_robotContainer.getM_robotDrive().configBrakeMode(true);
    m_timerCheck = false;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    outputToShuffleBoard();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getM_robotDrive().configBrakeMode(true);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonCommandOneTimer = Timer.getFPGATimestamp();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    new IntakeIn(m_robotContainer.getM_intake());
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(Timer.getFPGATimestamp() - m_autonCommandOneTimer >= 9.3 && m_timerCheck ==false){
//      m_autonomousCommand.cancel();
//      m_autonomousCommandTwo.schedule();
////      System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX "+m_robotContainer.getM_robotDrive().getPose().getX() +" "+ m_robotContainer.getM_robotDrive().getPose().getY() + " " + m_robotContainer.getM_robotDrive().getPose().getRotation().getRadians());
//      m_robotContainer.getM_robotDrive().shift(true);
      m_timerCheck = true;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommandTwo.cancel();
    }
    m_robotContainer.getM_robotDrive().configBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void outputToShuffleBoard(){
    m_robotContainer.getM_robotDrive().outputToShuffleBoard();
  }
}