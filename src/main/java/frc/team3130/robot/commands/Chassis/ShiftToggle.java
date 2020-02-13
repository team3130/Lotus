package frc.team3130.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.subsystems.ExampleSubsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.Timer;

import java.util.Set;

public class ShiftToggle implements Command {
    private final Set<Subsystem> subsystems;

    public ShiftToggle() {
        this.subsystems = Set.of(Chassis.getInstance());
        timer = new Timer();
        hasShifted = false;
        currentShift = Chassis.isLowGear();
    }
    private Timer timer;
    private boolean hasShifted;
    private boolean currentShift;

    @Override
    public void initialize() {
        timer.reset();
        Chassis.driveTank(0, 0, false); 		//Cut all power to the motors so they aren't running during the shift
        timer.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if(!hasShifted && timer.get() > RobotMap.kChassisShiftWait){
            currentShift = Chassis.isLowGear();

            Chassis.shift(!currentShift); //toggle the gear to what it isn't currently
            hasShifted = true;

            //Reset the timer so that the ending dead time is from shifting rather than from the start.
            timer.reset();
            timer.start();
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
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (hasShifted && timer.get() > RobotMap.kChassisShiftWait);
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
        hasShifted = false;
        timer.stop();
    }

    /**
     * <p>
     * Specifies the set of subsystems used by this command.  Two commands cannot use the same
     * subsystem at the same time.  If the command is scheduled as interruptible and another
     * command is scheduled that shares a requirement, the command will be interrupted.  Else,
     * the command will not be scheduled. If no subsystems are required, return an empty set.
     * </p><p>
     * Note: it is recommended that user implementations contain the requirements as a field,
     * and return that field here, rather than allocating a new set every time this is called.
     * </p>
     *
     * @return the set of subsystems that are required
     */
    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
