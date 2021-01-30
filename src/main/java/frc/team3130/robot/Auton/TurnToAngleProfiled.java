package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;


/**
 * A command that will turn the robot to the specified angle using a motion profile.
 */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
    /**
     * Turns to robot to the specified angle using a motion profile.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */
    public TurnToAngleProfiled(double targetAngleDegrees, Chassis drive) {
        super(
                new ProfiledPIDController(RobotMap.kMPChassisP, RobotMap.kMPChassisI,
                        RobotMap.kMPChassisD, new TrapezoidProfile.Constraints(
                        RobotMap.kMaxTurnThrottle,
                        RobotMap.kMaxTurnAccelerationDegPerSSquared)),
                // Close loop on heading
                drive::getHeading,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                (output, setpoint) -> drive.arcadeDrive(0, output),
                // Require the drive
                drive);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
                .setTolerance(RobotMap.kTurnToleranceDeg, RobotMap.kTurnRateToleranceDegPerS);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atGoal();
    }
}
