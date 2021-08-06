package frc.team3130.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.RobotContainer;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

import java.util.function.DoubleSupplier;

public class GoToBal extends CommandBase {
    private final Chassis m_chassis;

    public GoToBal(Chassis subsystem) {
        m_chassis = subsystem;
        m_requirements.add(m_chassis);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_chassis.configRampRate(RobotMap.kDriveMaxRampRate);

    }
    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double moveSpeed, turnSpeed;

        if (RobotMap.balRelativePos[1] < 0) {
            moveSpeed = -(RobotMap.balRelativePos[1] / RobotMap.kYHeight);
        }

        else if (RobotMap.balRelativePos[1] > 0) {
            moveSpeed = RobotMap.balRelativePos[1] / RobotMap.kYHeight;
        }

        else {
            moveSpeed = 0;
        }

        if (RobotMap.balRelativePos[0] < 0) {
            turnSpeed = -(RobotMap.balRelativePos[0] / RobotMap.kXWidth);
        }

        else if (RobotMap.balRelativePos[0] > 0) {
            turnSpeed = RobotMap.balRelativePos[0] / RobotMap.kXWidth;
        }

        else {
            turnSpeed = 0;
        }

        m_chassis.driveArcade(moveSpeed, turnSpeed * RobotMap.kMaxTurnThrottle, true);
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
        if (RobotMap.balRelativePos[0] == RobotMap.kXWidth + 1 || RobotMap.balRelativePos[1] == RobotMap.kYHeight + 1) {
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
