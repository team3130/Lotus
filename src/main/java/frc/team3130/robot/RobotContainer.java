package frc.team3130.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3130.robot.IntakeCommand.IntakeOut;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

import static edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link frc.team3130.robot.Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


    RamseteCommand bounceRamseteCommandOne, bounceRamseteCommandTwo, bounceRamseteCommandThree, bounceRamseteCommandFour;

    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    private SendableChooser f = new SendableChooser();

    private ArrayList<Trajectory> m_bounceTrajectories= new ArrayList();
    private ArrayList<RamseteCommand> m_ramseteCommands = new ArrayList(Arrays.asList(bounceRamseteCommandOne,bounceRamseteCommandTwo,bounceRamseteCommandThree,bounceRamseteCommandFour));



    // The robot's subsystems
    private final frc.team3130.robot.DriveSubsystem m_robotDrive = new frc.team3130.robot.DriveSubsystem();
    private final Intake m_intake = new Intake();

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
//        m_intake.setDefaultCommand(
//                new IntakeIn(m_intake)
//        );

    }

    public static Trajectory getTrajectoryTwo(){
        String trajectoryTwoJSON = "/home/lvuser/deploy/output/" + "BarrelRacingTwo" + ".wpilib.json";
        Trajectory trajectoryTwo = new Trajectory();
        try {
            Path trajectoryPathTwo = Filesystem.getDeployDirectory().toPath().resolve(trajectoryTwoJSON);
            trajectoryTwo = TrajectoryUtil.fromPathweaverJson(trajectoryPathTwo);
        } catch (IOException ex) {
        }
        return trajectoryTwo;
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
                                frc.team3130.robot.RobotMap.lowGearkS,
                                frc.team3130.robot.RobotMap.lowGearkV,
                                frc.team3130.robot.RobotMap.LowGearkA),
                        m_robotDrive.getM_kinematics(),
                        12);


        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        4,
                        4)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_robotDrive.getM_kinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        config.setReversed(false);


        //NON BOUNCE PATH
        String trajectoryJSON = "/home/lvuser/deploy/output/" + "SnowPlow" + ".wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }



        m_robotDrive.resetOdometry(trajectory.getInitialPose());


        //BOUNCE PATH

//        String trajectoryJSON = "/home/lvuser/deploy/output/" + "Bounce1" + ".wpilib.json";
//        Trajectory firstTrajectory = new Trajectory();
//        try {
//            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//            firstTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//        } catch (IOException ex) {
//            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//        }
//        m_robotDrive.resetOdometry(firstTrajectory.getInitialPose());
//        m_bounceTrajectories.add(firstTrajectory);
//
//
//        Trajectory secondTrajectory =
//                TrajectoryGenerator.generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        new Pose2d(2.26, 3.5, new Rotation2d(Math.toRadians(135))),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        List.of(new Translation2d(2.89, 1.326), new Translation2d(3.7687, 0.644), new Translation2d(4.5, .644)),
//                        // End 3 meters straight ahead of where we started, facing forward
//                        new Pose2d(4.9, 3.6, new Rotation2d(Math.toRadians(280))),
//                        // Pass config
//
//                        config);
//        m_bounceTrajectories.add(secondTrajectory);
//
//
//        trajectoryJSON = "/home/lvuser/deploy/output/" + "Bounce2" + ".wpilib.json";
//        Trajectory thirdTrajectory = new Trajectory();
//        try {
//            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//            thirdTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//        } catch (IOException ex) {
//            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//        }
//        m_bounceTrajectories.add(thirdTrajectory);
//
//
//        Trajectory fourthTrajectory =
//                TrajectoryGenerator.generateTrajectory(
//                                new Pose2d(7.4, 3.51, new Rotation2d(Math.toRadians(461))),
//                        List.of(new Translation2d(7.5,3.2)),
//                                new Pose2d(7.9, 2.9, new Rotation2d(Math.toRadians(560))),
//                        config);
//
//        m_bounceTrajectories.add(fourthTrajectory);
//
//
//        for(int x=0; x<4;x++) {
//            m_ramseteCommands.set(x,
//                    new RamseteCommand(
//                            m_bounceTrajectories.get(x),
//                            m_robotDrive::getPose,
//                            new RamseteController(2.0, .7),
//                            new SimpleMotorFeedforward(
//                                    frc.team3130.robot.RobotMap.lowGearkS,
//                                    frc.team3130.robot.RobotMap.lowGearkV,
//                                    frc.team3130.robot.RobotMap.LowGearkA),
//                            m_robotDrive.getM_kinematics(),
//                            m_robotDrive::getWheelSpeeds,
//                            new PIDController(2.05, 0, 0),
//                            new PIDController(2.05, 0, 0),
//                            // RamseteCommand passes volts to the callback
//                            m_robotDrive::tankDriveVolts,
//                            m_robotDrive)
//            );
//        }

//                Trajectory TrajectoryTwo =
//                generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        List.of(new Pose2d(7, 2.56, new Rotation2d(Math.toRadians(167))),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        // End 3 meters straight ahead of where we started, facing forward
//                        new Pose2d(0, 2, new Rotation2d(Math.toRadians(160)))),
//
//
//                        config);
//
        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        trajectory,
                        m_robotDrive::getPose,
                        new RamseteController(2.0, .7),
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.lowGearkS,
                                frc.team3130.robot.RobotMap.lowGearkV,
                                frc.team3130.robot.RobotMap.LowGearkA),
                        m_robotDrive.getM_kinematics(),
                        m_robotDrive::getWheelSpeedsLowGear,
                        new PIDController(2.05, 0, 0),
                        new PIDController(2.05, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);
//
//        RamseteCommand ramseteCommandTwo =
//                new RamseteCommand(
//                        TrajectoryTwo,
//                        m_robotDrive::getPose,
//                        new RamseteController(2.0, .7),
//                        new SimpleMotorFeedforward(
//                                frc.team3130.robot.RobotMap.highGearkS,
//                                frc.team3130.robot.RobotMap.highGearkV,
//                                frc.team3130.robot.RobotMap.highGearkA),
//                        m_robotDrive.getM_kinematics(),
//                        m_robotDrive::getWheelSpeedsHighGear,
//                        new PIDController(.69, 0, 0),
//                        new PIDController(.69, 0, 0),
//                        // RamseteCommand passes volts to the callback
//                        m_robotDrive::tankDriveVolts,
//                        m_robotDrive);

//
//
//        SequentialCommandGroup m_commandGroup = new SequentialCommandGroup(m_ramseteCommands.get(0),m_ramseteCommands.get(1),m_ramseteCommands.get(2),m_ramseteCommands.get(3));


        // Run path following command, then stop at the end.
//        return m_commandGroup.andThen(() -> m_robotDrive.configBrakeMode(true));

//        IntakeIn runIntake =new IntakeIn(m_intake);
        Shift shift = new Shift(m_robotDrive);
        ResetOdometry resetOdometry = new ResetOdometry(m_robotDrive);



        SequentialCommandGroup m_sequntialCommandGroup = new SequentialCommandGroup(ramseteCommand);

        return m_sequntialCommandGroup; //.andThen(() -> m_robotDrive.configBrakeMode(true));
    }


    public Command getSecondAutonomousCommand(){
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.lowGearkS,
                                frc.team3130.robot.RobotMap.lowGearkV,
                                frc.team3130.robot.RobotMap.LowGearkA),
                        m_robotDrive.getM_kinematics(),
                        12);


        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        4,
                        4)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_robotDrive.getM_kinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        config.setReversed(false);

        Trajectory TrajectoryTwo =
                generateTrajectory(
                        // Start at the origin facing the +X direction
                        List.of(new Pose2d(7.537, 2.384, new Rotation2d(2.955)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(0, 3.7, new Rotation2d(Math.toRadians(180)))),


                        config);
        RamseteCommand ramseteCommandTwo =
                new RamseteCommand(
                        TrajectoryTwo,
                        m_robotDrive::getPose,
                        new RamseteController(2.0, .7),
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.highGearkS,
                                frc.team3130.robot.RobotMap.highGearkV,
                                frc.team3130.robot.RobotMap.highGearkA),
                        m_robotDrive.getM_kinematics(),
                        m_robotDrive::getWheelSpeedsHighGear,
                        new PIDController(.69, 0, 0),
                        new PIDController(.69, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);

        Shift shift = new Shift(m_robotDrive);
        ResetOdometry resetOdometry = new ResetOdometry(m_robotDrive);

        SequentialCommandGroup m_sequntialCommandGroup = new SequentialCommandGroup(shift,resetOdometry, ramseteCommandTwo);

        return m_sequntialCommandGroup.andThen(() -> m_robotDrive.configBrakeMode(true));
    }

    public frc.team3130.robot.DriveSubsystem getM_robotDrive() {
        return m_robotDrive;
    }

    public Intake getM_intake(){
        return m_intake;
    }
}
