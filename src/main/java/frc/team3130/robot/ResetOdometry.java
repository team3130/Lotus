package frc.team3130.robot;

        import edu.wpi.first.wpilibj.DriverStation;
        import edu.wpi.first.wpilibj.Filesystem;
        import edu.wpi.first.wpilibj.geometry.Pose2d;
        import edu.wpi.first.wpilibj.geometry.Rotation2d;
        import edu.wpi.first.wpilibj.trajectory.Trajectory;
        import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
        import edu.wpi.first.wpilibj2.command.CommandBase;
        import frc.team3130.robot.Intake;

        import java.io.IOException;
        import java.nio.file.Path;

public class ResetOdometry extends CommandBase {
    private final DriveSubsystem m_drive;


    public ResetOdometry(DriveSubsystem subsystem) {
        m_drive = subsystem;

        m_requirements.add(m_drive);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

//        m_drive.resetOdometry(new Pose2d(7, 2.56, new Rotation2d(Math.toRadians(167))));
        m_drive.resetOdometry(new Pose2d(7.537, 2.384, new Rotation2d(2.955)));
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
        return true;
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

    }
}
