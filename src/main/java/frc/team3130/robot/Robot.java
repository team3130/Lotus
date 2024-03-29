package frc.team3130.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3130.robot.sensors.Navx;
import frc.team3130.robot.sensors.vision.Limelight;
import frc.team3130.robot.sensors.vision.WheelSpeedCalculations;


import static frc.team3130.robot.RobotContainer.m_driverGamepad;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public RobotContainer m_robotContainer;

    CommandScheduler scheduler = CommandScheduler.getInstance();
    CommandBase autonomousCommand = null;
    private SendableChooser<String> chooser = new SendableChooser<String>();
    private static Timer timer;
    private static double lastTimestamp;

    boolean gettime = true;
    boolean checkif = true;


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        timer = new Timer();
        timer.reset();
        timer.start();

        //Instantiate operator interface
        m_robotContainer = new RobotContainer();

        //Instantiate Limelight interface
        Limelight.GetInstance();

        //Instantiate Navx
        Navx.GetInstance();

        //Instantiate Wheel Speed interpolator
        WheelSpeedCalculations.GetInstance();


        Limelight.GetInstance().setLedState(false); //Turn vision tracking off when robot boots up

    }

    @Override
    public void disabledInit() {
        m_robotContainer.getChassis().configBrakeMode(true);
        m_robotContainer.getIntake().retractIntake();
        //Hood.setPistons(false);
        m_robotContainer.getWOF().retractWheel();
        m_robotContainer.getClimber().retractClimb();
        Limelight.GetInstance().setLedState(false); //Turn vision tracking off when robot disables
        m_robotContainer.getHood().setPiston(false);
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        outputToShuffleboard();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        determineAuto();

        // Schedule autonomous command if it exists
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Limelight.GetInstance().updateData(m_robotContainer.getTurret());
        scheduler.run();
        writePeriodicOutputs();
    }

    @Override
    public void teleopInit() {
        /*
        This makes sure that the autonomous stops running when
         teleop starts running. If you want the autonomous to
         continue until interrupted by another command, remove
         this line or comment it out.
         */
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        m_robotContainer.getChassis().configBrakeMode(true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Limelight.GetInstance().updateData(m_robotContainer.getTurret());
        scheduler.run();
        writePeriodicOutputs();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Updates shuffleboard periodically
     * WARNING: outputting to many things to shuffleboard overloads Rio and causes robot latency
     */
    public void outputToShuffleboard() {
//        Navx.GetInstance().outputToShuffleboard();
//        WheelOfFortune.outputToShuffleboard();
//        Chassis.outputToShuffleboard();
        m_robotContainer.getHopper().outputToShuffleboard();
        m_robotContainer.getTurret().outputToShuffleboard();
//        Hopper.outputToShuffleboard();
        Limelight.GetInstance().outputToShuffleboard(m_robotContainer.getTurret());
        m_robotContainer.getFlywheel().outputToShuffleboard();
//        HoodAngleCalculations.GetInstance().outputToShuffleboard();
        m_robotContainer.getHood().outputToShuffleboard();

        //TODO: move this somewhere logical
        if (RobotState.isEnabled() && m_robotContainer.getTurret().isOnTarget() && checkif) {
            if (gettime) {
                lastTimestamp = Timer.getFPGATimestamp();
                gettime = false;
            }
            m_driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 1);
            m_driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            if (Timer.getFPGATimestamp() - lastTimestamp > .3) {
                checkif = false;
            }
        } else {
            m_driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
            m_driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            gettime = true;
            if (!m_robotContainer.getTurret().isOnTarget()) {
                checkif = true;
            }
        }


    }

    private void determineAuto() {

    }

    public void writePeriodicOutputs() {
        m_robotContainer.getTurret().writePeriodicOutputs();
    }

    public void resetSubsystems() {

    }
}
