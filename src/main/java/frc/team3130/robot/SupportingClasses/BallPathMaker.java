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

import java.util.ArrayList;

public class BallPathMaker extends ComputerVision implements Runnable{
    private final Chassis m_chassis;
    private PixyCam pixy;
    private RamseteCommand cmd;

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

    public void makeCmd() {

        ArrayList<Pixy2CCC.Block> blocks = pixy.getBlocks();

        m_chassis.configRampRate(RobotMap.kDriveMaxRampRate);

        // TODO: logic for sanity check
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

        ArrayList<Pose2d> route = new ArrayList<>();

        route.add(m_chassis.getPose());

        double[] coords;

        for (Pixy2CCC.Block block : blocks) {
            coords = predict(new double[]{block.getX(), block.getY()});
            route.add(new Pose2d(coords[0] + m_chassis.getPose().getX(), coords[1] + m_chassis.getPose().getY(), new Rotation2d()));
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(route, config);

        cmd = new RamseteCommand(
                trajectory,
                m_chassis::getPosCV,
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

    public RamseteCommand getCmd() {
        System.out.println("Ramsete command is still null");
        return cmd;
    }

    @Override
    public void run() {
        makeCmd();
    }
}
