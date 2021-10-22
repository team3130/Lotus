package frc.team3130.robot.SupportingClasses;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
import java.util.List;

/**
 * <p>
 *     Extends the ComputerVision class
 *      This class is intended to be ran in it's own thread
 * </p>
 */
public class PosCV extends ComputerVision implements Runnable{
    private final Chassis m_chassis;
    private PixyCam pixy;
    private Pose2d position;

    /**
     *<p>instantiates the PosCV class with the chassis subsystem and an instance of pixy</p>
     * @param m_chassis chassis subsystem
     * @param pixy pixy class
     */
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

    /**
     *<p>getter for position</p>
     * @return position on field as a Pose2d object
     */
    public Pose2d getPos() {
        double[] pos = {pixy.getBlocks().get(0).getX(), pixy.getBlocks().get(0).getY()};
        position = new Pose2d(new Translation2d(predict(pos)), new Rotation2d(pixy.getBlocks().get(0).getAngle()));
        return position;
    }

    @Override
    public void run() {
        getPos();
    }
}
