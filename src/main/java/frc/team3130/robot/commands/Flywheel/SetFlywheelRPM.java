package frc.team3130.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.subsystems.Climber;
import frc.team3130.robot.subsystems.Flywheel;
import frc.team3130.robot.subsystems.Hood;
import frc.team3130.robot.vision.Limelight;
import frc.team3130.robot.vision.WheelSpeedCalculations;

import java.util.Set;

public class SetFlywheelRPM extends CommandBase {
    private final Flywheel m_flywheel;
    private final Hood m_hood;

    public SetFlywheelRPM(Flywheel subsystem1, Hood subsystem2) {
        m_flywheel = subsystem1;
        m_hood = subsystem2;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        if (!Limelight.GetInstance().hasTrack()){
            m_flywheel.setSpeed(3500.0);
        }else {
            double x = Limelight.GetInstance().getDistanceToTarget();
            if (71.0 <= x) {
                //Hood.setPistons(false);
                double speed = WheelSpeedCalculations.GetInstance().getSpeed(x);
                m_flywheel.setSpeed(speed);
            } else{
                m_flywheel.setSpeed(3500);
            }
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
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
    }
}
