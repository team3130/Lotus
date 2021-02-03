package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
import java.util.ArrayList;
import java.util.function.Supplier;

public class AutoChooser {
    private ArrayList<PathsInterface> m_chooser;
    private PathsInterface m_path;
    private RamseteCommand command;

    public AutoChooser() {
        this.m_chooser = new ArrayList<PathsInterface>();
        m_chooser.add(new SlalomPathsInterface());
        m_chooser.add(new galacticSearchARed());
        m_chooser.add(new galacticSearchABlue());
        m_chooser.add(new galacticSearchBRed());
        m_chooser.add(new galacticSearchBBlue());
        m_chooser.add(new BarrelPoints());
        m_chooser.add(new BouncePaths());
        this.m_path = null;
    }

    public void determinePath(int path) {
        this.m_path = this.m_chooser.get(path);
    }
    public Command getCommand(Chassis m_chassis, int path) {
        this.determinePath(path);
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(RobotMap.kMaxVelocityPerSecond),
                Units.feetToMeters(RobotMap.kMaxAccelerationPerSecond));

        config.setKinematics(m_chassis.getmKinematics());

        command = new RamseteCommand(
                this.m_path.getWaypoints(),
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

    private class SlalomPathsInterface implements PathsInterface {
        private Trajectory slathom;

        public SlalomPathsInterface() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/Slalom.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.slathom = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.slathom;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchARed implements PathsInterface {
        private Trajectory galacticSearch;

        public galacticSearchARed() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/GalacticSearchARed.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }

            this.galacticSearch = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchABlue implements PathsInterface {
        private Trajectory galacticSearch;

        public galacticSearchABlue() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/GalacticSearchABlue.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.galacticSearch = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchBRed implements PathsInterface {
        private Trajectory galacticSearch;

        public galacticSearchBRed() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/GalacticSearchBRed.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.galacticSearch = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchBBlue implements PathsInterface {
        private Trajectory galacticSearch;

        public galacticSearchBBlue() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/GalacticSearchBBlue";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.galacticSearch = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class BarrelPoints implements PathsInterface {
        private Trajectory BarrelPoints;

        public BarrelPoints() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/BarrelRacing.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.BarrelPoints = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.BarrelPoints;
        }

        @Override
        public void Start() {

        }
    }

    private class BouncePaths implements PathsInterface {
        private Trajectory Bounce;

        public BouncePaths() {
            String trajectoryJSON = "/home/lvuser/deploy/paths/Bounce.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            this.Bounce = trajectory;
        }

        @Override
        public Trajectory getWaypoints() {
            return this.Bounce;
        }

        @Override
        public void Start() {

        }
    }

}
