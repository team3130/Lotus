package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.subsystems.Chassis;

public class AutoTurn extends CommandBase {

    private double angle;
    private double thresh;
    private boolean smallAngle;

    /**
     * Creates a new AutoTurn.
     */
    public AutoTurn() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Chassis.getInstance());
    }

    /**
     * Sets the turn parameters
     *
     * @param angle     in degrees
     * @param threshold in degrees
     */
    public void setParam(double angle, double threshold) {
        this.angle = angle;
        thresh = threshold;
        smallAngle = false;
    }

    public void setParam(double angle, double threshold, boolean smallAngle) {
        setParam(angle, threshold);
        this.smallAngle = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("StartAutoTurn");
        Chassis.shift(false);
        Chassis.ReleaseAngle();
        Chassis.getInstance().setAbsoluteTolerance(thresh);
        Chassis.holdAngle(angle, smallAngle);
        Chassis.driveStraight(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Angle: " + Chassis.getAngle());
        System.out.println("Setpoint: " + Chassis.getInstance().getSetpoint());
        System.out.println();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING");
        Chassis.ReleaseAngle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(Chassis.getAngle() - Chassis.getInstance().getSetpoint()) < thresh;
    }
}