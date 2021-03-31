package frc.team3130.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link frc.team3130.robot.Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {



    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    private NetworkTableEntry   P =
            tab.add("Chassis proportional", 2.05).getEntry();



    // The robot's subsystems
    private final frc.team3130.robot.DriveSubsystem m_robotDrive = new frc.team3130.robot.DriveSubsystem();

    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    // The driver's controller
//    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        configureButtonBindings();
        m_robotDrive.setDefaultCommand(
                new frc.team3130.robot.DefaultDrive(
                        m_robotDrive,
                        () -> m_driverGamepad.getY(GenericHID.Hand.kLeft),
                        () -> m_driverGamepad.getX(GenericHID.Hand.kRight)
                )
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
//        // Drive at half speed when the right bumper is held
//        new JoystickButton(m_driverController, Button.kBumperRight.value)
//                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
//                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    /**
     * Use this to pass the autonomous command to the main  class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.kS,
                                frc.team3130.robot.RobotMap.kV,
                                frc.team3130.robot.RobotMap.kA),
                        m_robotDrive.getM_kinematics(),
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        3,
                        3)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_robotDrive.getM_kinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

//         An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config

                        config);

//        String trajectoryJSON = "/home/lvuser/deploy/output/" + "Bounce" + ".wpilib.json";
//        Trajectory exampleTrajectory = new Trajectory();
//        try {
//            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//        } catch (IOException ex) {
//            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//        }
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        m_robotDrive::getPose,
                        new RamseteController(2.0, .7),
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.kS,
                                frc.team3130.robot.RobotMap.kV,
                                frc.team3130.robot.RobotMap.kA),
                        m_robotDrive.getM_kinematics(),
                        m_robotDrive::getWheelSpeeds,
                        new PIDController(.599,0,0),
                        new PIDController(.599,0,0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.


        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.configBrakeMode(true));
    }


    public frc.team3130.robot.DriveSubsystem getM_robotDrive() {
        return m_robotDrive;
    }
}
