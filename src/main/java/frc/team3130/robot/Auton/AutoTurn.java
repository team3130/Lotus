package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.subsystems.Chassis;

public class AutoTurn extends CommandBase {
    private final Chassis m_chassis;

    private double angle;
    private double thresh;
    private boolean smallAngle;

    /**
     * Creates a new AutoTurn.
     */
    public AutoTurn(Chassis subsystem) {
        m_chassis = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_chassis);
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
        Chassis.ReleaseAngle(m_chassis);
        m_chassis.setAbsoluteTolerance(thresh);
        Chassis.holdAngle(angle, smallAngle, m_chassis);
        Chassis.driveStraight(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Angle: " + Chassis.getAngle());
        System.out.println("Setpoint: " + m_chassis.getSetpoint());
        System.out.println();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING");
        Chassis.ReleaseAngle(m_chassis);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(Chassis.getAngle() - m_chassis.getSetpoint()) < thresh;
    }
}