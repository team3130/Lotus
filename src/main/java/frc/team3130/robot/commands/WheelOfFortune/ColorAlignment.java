package frc.team3130.robot.commands.WheelOfFortune;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.GameData;
import frc.team3130.robot.subsystems.WheelOfFortune;

import java.util.Set;

public class ColorAlignment extends CommandBase {
    private final WheelOfFortune m_wof;

    private static String fieldColor;
    private static String targetColor;
    private static double motorSpeed = .3;


    //used to terminate the program
    private static boolean colorFound;

    public ColorAlignment(WheelOfFortune subsystem) {
        m_wof = subsystem;
        colorFound = false;
        m_requirements.add(m_wof);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        colorFound = false;
        fieldColor = GameData.getInstance().getControlColor();
        targetColor = m_wof.getTargetColor(fieldColor, m_wof);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        String color = m_wof.determineColor();

        if (color.equals(targetColor)){
            m_wof.motorSpin(0);
            System.out.println("color found");

        } else if(color.equals("Red")) {
//            System.out.println("Looking at Red");
            switch (targetColor) {
                case "Blue":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Green":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Yellow":
                    m_wof.motorSpin(-motorSpeed);
                    break;
            }

        } else if (color.equals("Green")) {
//            System.out.println("Looking at Green");
            switch (targetColor) {
                case "Red":
                    m_wof.motorSpin(-motorSpeed);
                    break;
                case "Blue":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Yellow":
                    m_wof.motorSpin(motorSpeed);
                    break;
            }
        } else if (color.equals("Blue")){
//            System.out.println("Looking at " + color);
            switch (targetColor) {
                case "Red":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Green":
                    m_wof.motorSpin(-motorSpeed);
                    break;
                case "Yellow":
                    m_wof.motorSpin(motorSpeed);
                    break;
            }
        } else if (color.equals("Yellow")){
//            System.out.println("Looking at Yellow");
            switch (targetColor) {
                case "Red":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Green":
                    m_wof.motorSpin(motorSpeed);
                    break;
                case "Blue":
                    m_wof.motorSpin(-motorSpeed);
                    break;
            }
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
        String color = m_wof.determineColor();
        //Code should turn off now
        if (color.equals(targetColor)) {
            m_wof.motorSpin(0);
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

    }
}
