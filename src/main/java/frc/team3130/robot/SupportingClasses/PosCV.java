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

public class PosCV extends ComputerVision implements Runnable{
    private final Chassis m_chassis;
    private PixyCam pixy;
    private Pose2d position;

    public PosCV(Chassis m_chassis, PixyCam pixy) {
        this.m_chassis = m_chassis;
        this.pixy = pixy;

        //TODO: find the real values
        rotation = new double[][]{
                {-0.06815984, -0.87361327},
                {-0.38507791, 0.14232387}
        };

        translation = new double[]{216.85726298, 63.07897139};
    }

    public Pose2d getPos() {
        return position;
    }

    @Override
    public void run() {
        getPos();
    }
}
