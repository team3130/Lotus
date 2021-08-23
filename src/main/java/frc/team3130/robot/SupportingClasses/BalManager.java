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
import frc.team3130.robot.subsystems.Chassis;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.ArrayList;

public class BalManager implements Runnable{
    private final Chassis m_chassis;
    private ArrayList<Pixy2CCC.Block> blocks;
    private RamseteCommand cmd;

    private final double[][] rotation = {
            {-0.06815984, -0.87361327},
            {-0.38507791,  0.14232387}
    };

    private final double[] translation = {216.85726298,  63.07897139};

    public BalManager(Chassis m_chassis, ArrayList<Pixy2CCC.Block> blocks) {
        this.m_chassis = m_chassis;
        this.blocks = blocks;
    }


    public double[] predict(double[] pixel) {
        double[] coords = new double[2];
        coords[0] = ((pixel[0] * rotation[0][0]) + (pixel[1] * rotation[0][1])) + translation[0];
        coords[1] = ((pixel[0] * rotation[1][0]) + (pixel[1] * rotation[1][1])) + translation[1];
        return coords;
    }

    public void makeCmd(ArrayList<Pixy2CCC.Block> blocks) {
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

    public RamseteCommand getCmd() throws InterruptedException {
        while (cmd == null) {
            System.out.println("Ramsete command is still null");
            this.wait();
        }
        return cmd;
    }

    @Override
    public void run() {
        makeCmd(blocks);
    }
}
