package frc.team3130.robot.commands.WheelOfFortune;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.subsystems.ExampleSubsystem;
import frc.team3130.robot.subsystems.WheelOfFortune;

import java.util.Set;

public class ColorAlignment implements Command {
    private final Set<Subsystem> subsystems;

    //TODO: make field color dependent on inputs from field
    private static String fieldColor;
    private static String targetColor;
    private static double motorSpeed = .2;


    //used to terminate the program
    private static boolean colorFound;

    public ColorAlignment() {
        this.subsystems = Set.of(WheelOfFortune.getInstance());
        colorFound = false;

    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        colorFound = false;
        fieldColor = "Red";
        targetColor = WheelOfFortune.getTargetColor(fieldColor);

        WheelOfFortune.deployWheel();

        System.out.println("Initialized");
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        String color = WheelOfFortune.determineColor();

        if (color.equals(targetColor)){
            WheelOfFortune.motorSpin(0);
            System.out.println("color found");

        } else if(color.equals("Red")) {
            System.out.println("Looking at Red");
            switch (targetColor) {
                case "Cyan":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Green":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Yellow":
                    WheelOfFortune.motorSpin(-motorSpeed);
                    break;
            }

        } else if (color.equals("Green")) {
            System.out.println("Looking at Green");
            switch (targetColor) {
                case "Red":
                    WheelOfFortune.motorSpin(-motorSpeed);
                    break;
                case "Cyan":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Yellow":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
            }
        } else if (color.equals("Cyan")){
            System.out.println("Looking at " + color);
            switch (targetColor) {
                case "Red":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Green":
                    WheelOfFortune.motorSpin(-motorSpeed);
                    break;
                case "Yellow":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
            }
        } else if (color.equals("Yellow")){
            System.out.println("Looking at Yellow");
            switch (targetColor) {
                case "Red":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Green":
                    WheelOfFortune.motorSpin(motorSpeed);
                    break;
                case "Cyan":
                    WheelOfFortune.motorSpin(-motorSpeed);
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
        String color = WheelOfFortune.determineColor();
        //Code should turn off now
        if (color.equals(targetColor)) {
            WheelOfFortune.motorSpin(0);
            WheelOfFortune.retractWheel();
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
