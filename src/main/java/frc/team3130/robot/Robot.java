// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3130.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3130.robot.vision.Limelight;
import frc.team3130.robot.vision.WheelSpeedCalculations;

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
    m_robotContainer.getM_robotDrive().configBrakeMode(true);
    m_timerCheck = false;

    Navx.GetInstance();

    //Instantiate Limelight interface
    Limelight.GetInstance();


    //Instantiate Wheel Speed interpolator
    WheelSpeedCalculations.GetInstance();

    Limelight.GetInstance().setLedState(false); //Turn vision tracking off when robot boots up

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

    Limelight.GetInstance().setLedState(false); //Turn vision tracking off when robot disables
    m_robotContainer.getM_hood().setAngle(0);
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

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Limelight.GetInstance().updateData( m_robotContainer.getM_turret());

    if(Timer.getFPGATimestamp()-m_autonCommandOneTimer > 8){
      m_robotContainer.getM_intake().retractIntake();
      m_robotContainer.getM_intake().runIntake(0);
    }
    else if (Timer.getFPGATimestamp()-m_autonCommandOneTimer > 1.2){
      m_robotContainer.getM_intake().deployIntake();
      m_robotContainer.getM_intake().runIntake(.7);
    }
    writePeriodicOutputs();

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
    m_robotContainer.getM_intake().retractIntake();
    m_robotContainer.getM_intake().runIntake(0);
    m_robotContainer.getM_robotDrive().configBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Limelight.GetInstance().updateData(m_robotContainer.getM_turret());
    writePeriodicOutputs();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void writePeriodicOutputs() {
    m_robotContainer.getM_turret().writePeriodicOutputs();
  }


  public void outputToShuffleBoard(){
    m_robotContainer.getM_robotDrive().outputToShuffleBoard();
    Limelight.GetInstance().outputToShuffleboard(m_robotContainer.getM_turret());
    m_robotContainer.getM_flyWheel().outputToShuffleboard();
//        HoodAngleCalculations.GetInstance().outputToShuffleboard();
    m_robotContainer.getM_hood().outputToShuffleboard();
    m_robotContainer.getM_turret().outputToShuffleboard();
    Navx.GetInstance().outputToShuffleboard();
  }
}