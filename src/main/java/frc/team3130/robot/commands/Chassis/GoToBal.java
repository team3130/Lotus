package frc.team3130.robot.commands.Chassis;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3130.robot.RobotContainer;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.SupportingClasses.Bal;
import frc.team3130.robot.SupportingClasses.BalManager;
import frc.team3130.robot.subsystems.Chassis;

import java.util.List;
import java.util.function.DoubleSupplier;

public class GoToBal extends CommandBase {
    private final Chassis m_chassis;
    private final BalManager m_balManager;

    public GoToBal(Chassis subsystem, BalManager balManager) {
        m_chassis = subsystem;
        m_requirements.add(m_chassis);
        m_balManager = balManager;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kDriveMaxRampRate);

        // TODO: logic for sanity check
        // Create config for trajectory

        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.lowGearkS,
                                frc.team3130.robot.RobotMap.lowGearkV,
                                frc.team3130.robot.RobotMap.LowGearkA),
                        m_chassis.getmKinematics(),
                        12);

        TrajectoryConfig config =
                new TrajectoryConfig(
                        4,
                        4)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_chassis.getmKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        config.setReversed(false);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(m_chassis.getPosCV(), m_balManager.pop().getPose()), config);
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                m_chassis::getPosCV,
                new RamseteController(2.0, 0.7),
                new SimpleMotorFeedforward(
                        RobotMap.lowGearkS,
                        RobotMap.lowGearkV,
                        RobotMap.LowGearkA),
                m_chassis.getmKinematics(),
                m_chassis::getWheelSpeedsLowGear,
                new PIDController(2.05, 0, 0),
                new PIDController(2.05, 0, 0),
                m_chassis::tankDriveVolts,
                m_chassis);
        ramseteCommand.schedule();
    }
    
    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {}

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
        if (m_balManager.getClosestBall().getPositionRel().get()[0] == RobotMap.kXWidth + 1 || m_balManager.getClosestBall().getPositionRel().get()[1] == RobotMap.kYHeight + 1) {
            System.out.println("bal does not exist");
            return true;
        }
        else {
            return false;
        }
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
        m_chassis.configRampRate(0);
    }
}
