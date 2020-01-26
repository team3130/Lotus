package frc.team3130.robot.commands.WheelOfFortune;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.subsystems.WheelOfFortune;

public class TripleSpinFinish implements Command {
    private final Set<Subsystem> subsystems;
    Timer timer = new Timer();
    private static int redCounter;
    private static boolean isChanged;
    private static boolean isCounted;
    private static double lastTimestamp;

    public TripleSpinFinish() {
        this.subsystems = Set.of(WheelOfFortune.getInstance());
        isChanged = false;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        redCounter = 0;
        WheelOfFortune.motorSpin(0.5);
    }



    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        //store returned color into local variable
        String color = WheelOfFortune.detectHSB();

        if (color.equals("Red")){
            if (!isChanged) {
                lastTimestamp = Timer.getFPGATimestamp();
                isChanged = true;
                isCounted = false;
            }else{
                if(Timer.getFPGATimestamp() - lastTimestamp > .2 && !isCounted){
                    redCounter++;
                    isCounted = true;
                }
            }
        }else{
            isChanged = false;
        }

        System.out.println(redCounter);
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
        return redCounter >= 7;
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
        WheelOfFortune.motorSpin(0.0);
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
