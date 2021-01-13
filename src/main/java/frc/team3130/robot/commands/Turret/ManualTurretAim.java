package frc.team3130.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Turret;
import frc.team3130.robot.util.Utils;

import java.util.Set;

import static frc.team3130.robot.RobotContainer.weaponsGamepad;

public class ManualTurretAim extends CommandBase {
    private final Turret m_turret;

    private boolean manualChanged = false;

    public ManualTurretAim(Turret subsystem) {
        m_turret = subsystem;
    }


    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        manualChanged = false;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double turnSpeed = -weaponsGamepad.getRawAxis(RobotMap.LST_AXS_RJOYSTICKX); //returns value from -1 to 1 of R X axis of gamepad.

        if (Math.abs(turnSpeed) >= RobotMap.kTurretManualDeadband){
            double moveSpeed = RobotMap.kTurretManualMultipler * Utils.applyDeadband(turnSpeed, RobotMap.kTurretManualDeadband);
            Turret.manualOp(moveSpeed);
            manualChanged = true;
        } else if (manualChanged){
            Turret.aim(false);
            manualChanged = false;
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
    }
}
