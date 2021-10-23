package frc.team3130.robot.commands.Chassis;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.SupportingClasses.BallPathMaker;
import frc.team3130.robot.sensors.PixyCam;
import frc.team3130.robot.subsystems.Chassis;

/**
 * <p>Command to have the robot go to the ball</p>
 * //TODO: Research whether or not DefaultDrive needs to be interrupted
 *
 */
public class GoToBal extends CommandBase {
    private final Chassis m_chassis;
    private BallPathMaker m_ballPathMaker;
    private RamseteCommand cmd;
    private Thread thread;
    private PixyCam pixy;

    /**
     * <p>Instantiates GoToBal command with {@link Chassis} and with an object from {@link PixyCam}</p>
     * @param subsystem Chassis
     * @param pixy PixyCam object
     */
    public GoToBal(Chassis subsystem, PixyCam pixy) {
        m_chassis = subsystem;
        this.pixy = pixy;
        m_requirements.add(m_chassis);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_ballPathMaker = new BallPathMaker(m_chassis, pixy);
        thread = new Thread(m_ballPathMaker);
        thread.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true)
     */
    @Override
    public void execute() {
        if (!thread.isAlive()) {
            cmd = m_ballPathMaker.getCmd();
        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return cmd != null;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_chassis.configRampRate(0);
        thread.interrupt();
        try {
            thread.join();
        } catch (InterruptedException e) {
            DriverStation.reportError("Interrupted Exception", RobotMap.debug);
        }
        m_ballPathMaker = null;
        cmd.schedule();
    }
}
