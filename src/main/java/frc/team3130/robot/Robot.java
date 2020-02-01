package frc.team3130.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3130.robot.commands.Chassis.DefaultDrive;
import frc.team3130.robot.commands.Climber.SkyWalker;
import frc.team3130.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    CommandBase autonomousCommand = null;
    private SendableChooser<String> chooser = new SendableChooser<String>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //Instantiate operator interface
        OI.GetInstance();

        //Register and instantiate subsystems (optionally with default commands)
        //Note: registerSubsystem is NOT needed if setDefaultCommand is used
        scheduler.setDefaultCommand(Chassis.getInstance(), new DefaultDrive());
        scheduler.setDefaultCommand(Climber.getInstance(), new SkyWalker());
        scheduler.registerSubsystem(Intake.getInstance());
        scheduler.registerSubsystem(Hopper.getInstance());
        scheduler.registerSubsystem(Turret.getInstance());
        scheduler.registerSubsystem(WheelOfFortune.getInstance());


        //Smartdash output thread
        Thread t = new Thread(() -> {
            while (!Thread.interrupted()) {
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                outputToSmartDashboard();
            }
        });
        t.start();
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
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void robotPeriodic() {
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
        resetSubsystems();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        scheduler.run();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        scheduler.run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public void outputToSmartDashboard() {
        WheelOfFortune.detectColor();
        Chassis.outputToSmartDashboard();
    }

    public void writePeriodicOutputs() {
    }

    public void resetSubsystems() {

    }
}
