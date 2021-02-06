package frc.team3130.robot.Auton;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

import java.io.IOException;
import java.nio.file.Path;

public class AutoChooser {
    private RamseteCommand command;
    private String[] paths;
    private int number = 0;
    private SimpleWidget widget;

    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    public AutoChooser() {
        widget = tab.add("Path", 1);
        this.paths = new String[]{"B1D2", "B1toB8", "BarrelRacing", "Bounce", "DriveStraight", "GalacticSearchABlue", "GalacticSearchARed", "GalacticSearchBBlue", "GalacticSearchBRed", "S", "Slalom"};
    }

    public Command getCommand(Chassis m_chassis) {

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(RobotMap.kMaxVelocityPerSecond),
                Units.feetToMeters(RobotMap.kMaxAccelerationPerSecond));

        config.setKinematics(m_chassis.getmKinematics());

        //here to be default in case exception needs handling
        Trajectory trajectory = this.GenerateTrajectory(this.paths[4]);

        try {
            trajectory = this.GenerateTrajectory(this.paths[this.number]);
        } catch (ArrayIndexOutOfBoundsException ref) {
            DriverStation.reportError("Invalid Path, defaulting to DriveStraight", ref.getStackTrace());
        }


        command = new RamseteCommand(
                trajectory,
                m_chassis::getPose,
                new RamseteController(2.0, 0.7),
                m_chassis.getFeedforward(),
                m_chassis.getmKinematics(),
                m_chassis::getSpeeds,
                m_chassis.getleftPIDController(),
                m_chassis.getRightPIDController(),
                m_chassis::setOutput,
                m_chassis
        );

        return command;
    }

    public Trajectory GenerateTrajectory(String file) {
        String trajectoryJSON = "/home/lvuser/deploy/paths/" + file + ".wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
    }

    public void outputToShuffleboard() {
        this.number = (int) widget.getEntry().getDouble(1.0);
    }

}
