package frc.team3130.robot.commands.WheelOfFortune;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.subsystems.WheelOfFortune;

import java.util.Set;

public class TripleSpinFinish extends CommandBase {
    private final WheelOfFortune m_wof;

    private static int blueCounter;
    private static String lastColor;

    public TripleSpinFinish(WheelOfFortune subsystem) {
        m_wof = subsystem;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        blueCounter = 0;
        lastColor = "none";
        WheelOfFortune.motorSpin(0.5);
    }


    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        //store returned color into local variable
        String color = m_wof.determineColor();

        if (!lastColor.equals(color) && color.equals("Blue")) {
            blueCounter++;
        }
        lastColor = color;
        System.out.println("lmao the blueCounter is " + blueCounter);
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
        if (blueCounter >= 7) {
            return true;
        }
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
        WheelOfFortune.motorSpin(0.0);
    }
}
