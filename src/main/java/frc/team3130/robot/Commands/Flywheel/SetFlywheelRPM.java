package frc.team3130.robot.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.subsystems.Flywheel;
import frc.team3130.robot.subsystems.Hopper;

public class SetFlywheelRPM extends CommandBase {
    private final Flywheel m_flywheel;
    private final Hopper m_hoppper;

    public SetFlywheelRPM(Flywheel subsystem1, Hopper subsystem2) {
        m_flywheel = subsystem1;
        m_hoppper = subsystem2;
        m_requirements.add(m_flywheel);
        m_requirements.add(m_hoppper);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

            m_flywheel.setSpeed(m_flywheel.getgoalFlyWheelSpeed());
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        m_hoppper.runHopperLeft(-0.5);
        m_hoppper.runHopperRight(-0.6);
        if(m_flywheel.canShoot()){
            m_hoppper.runHopperTop(.6);
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
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_flywheel.stop();
        m_hoppper.runHopperLeft(0.0);
        m_hoppper.runHopperRight(0.0);
        m_hoppper.runHopperTop(0.0);
    }
}
