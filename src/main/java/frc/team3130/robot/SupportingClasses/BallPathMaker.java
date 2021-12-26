package frc.team3130.robot.SupportingClasses;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.sensors.PixyCam;
import frc.team3130.robot.subsystems.Chassis;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.ArrayDeque;
import java.util.ArrayList;

/**
 * <p>The latest ComputerVision implementation</p>
 * <p>This class takes advantage of the runnable interface, to run the code inside it's own thread.</p>
 */
public class BallPathMaker extends ComputerVision implements Runnable{
    private final Chassis m_chassis;
    private PixyCam pixy;
    private RamseteCommand cmd;

    /**
     * <p>Constructs a BallPathMaker object and takes in {@link Chassis} and a {@link PixyCam} object</p>
     * @param m_chassis Chassis object
     * @param pixy PixyCam object
     */
    public BallPathMaker(Chassis m_chassis, PixyCam pixy) {
        this.m_chassis = m_chassis;
        this.pixy = pixy;

        //TODO: find the real values
        rotation = new double[][]{
                {-0.06815984, -0.87361327},
                {-0.38507791, 0.14232387}
        };

        translation = new double[]{216.85726298, 63.07897139};
    }

    /**
     * <p>Method that makes a command based off of PixyCam values. This method is intended to be ran in a thread, as it takes a substantial amount of time to generate commands on the robot.</p>
     */
    public void makeCmd() {

        m_chassis.configRampRate(RobotMap.kDriveMaxRampRate);

        // Create config for trajectory
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                frc.team3130.robot.RobotMap.lowGearkS,
                                frc.team3130.robot.RobotMap.lowGearkV,
                                frc.team3130.robot.RobotMap.LowGearkA),
                        m_chassis.getmKinematics(),
                        12);

        TrajectoryConfig config =
                new TrajectoryConfig(
                        4,
                        4)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_chassis.getmKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        config.setReversed(false);

        // the 5 is the number of balls that it can still take
        //TODO: find a way to find out how many more balls we can hold
        ArrayDeque<Node> routeNode = Graph.getInstance().getPath(5);
        ArrayList<Pose2d> route = new ArrayList<>();

        for (Node node : routeNode) {
            route.add(node.getPos());
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(route, config);

        cmd = new RamseteCommand(
                trajectory,
                m_chassis::getPose,
                new RamseteController(2.0, 0.7),
                new SimpleMotorFeedforward(
                        RobotMap.lowGearkS,
                        RobotMap.lowGearkV,
                        RobotMap.LowGearkA),
                m_chassis.getmKinematics(),
                m_chassis::getWheelSpeedsLowGear,
                new PIDController(2.05, 0, 0),
                new PIDController(2.05, 0, 0),
                m_chassis::tankDriveVolts,
                m_chassis);
    }

    /**
     * <p>getter for the ramsete command that is being generated in {@link #makeCmd()}</p>
     * @return Ramsete command cmd
     */
    public RamseteCommand getCmd() {
        return cmd;
    }

    /**
     * <p>run method that gets called when the thread starts up</p>
     */
    @Override
    public void run() {
        makeCmd();
    }
}
