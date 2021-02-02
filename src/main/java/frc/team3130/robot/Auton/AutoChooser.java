package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

public class AutoChooser {
    private ArrayList<Path> m_chooser;
    private Path m_path;
    private RamseteCommand command;

    public AutoChooser() {
        this.m_chooser = new ArrayList<Path>();
        m_chooser.add(new SlathomPath());
        m_chooser.add(new galacticSearchARed());
        m_chooser.add(new galacticSearchBRed());
        m_chooser.add(new BarrelPoints());
        m_chooser.add(new BouncePath());
        this.m_path = null;
    }

    public void determinePath(int path) {
        this.m_path = this.m_chooser.get(path);
        this.m_path.Start();
    }
    public Command getCommand(Chassis m_chassis, int path) {
        this.determinePath(path);
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(RobotMap.kMaxVelocityPerSecond),
                Units.feetToMeters(RobotMap.kMaxAccelerationPerSecond));

        config.setKinematics(m_chassis.getmKinematics());

        //TODO: make it generate a trajectory based off of this.m_path.getWaypoints();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
                config
        );

        command = new RamseteCommand(
                trajectory,
                m_chassis::getPose,
                new RamseteController(2.0, 0.7),
                m_chassis.getFeedforward(),
                m_chassis.getmKinematics(),
                (Supplier<DifferentialDriveWheelSpeeds>) m_chassis.getSpeeds(),
                m_chassis.getleftPIDController(),
                m_chassis.getRightPIDController(),
                m_chassis::setOutput
        );

        return command;
    }

    private class SlathomPath implements Path {
        private Translation2d[] slathom;

        public SlathomPath() {
            this.slathom = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.slathom;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchARed implements Path {
        private Translation2d[] galacticSearch;

        public galacticSearchARed() {
            this.galacticSearch = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchABlue implements Path {
        private Translation2d[] galacticSearch;

        public galacticSearchABlue() {
            this.galacticSearch = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchBRed implements Path {
        private Translation2d[] galacticSearch;

        public galacticSearchBRed() {
            this.galacticSearch = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class galacticSearchBBlue implements Path {
        private Translation2d[] galacticSearch;

        public galacticSearchBBlue() {
            this.galacticSearch = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.galacticSearch;
        }

        @Override
        public void Start() {

        }
    }

    private class BarrelPoints implements Path {
        private Translation2d[] BarrelPoints;

        public BarrelPoints() {
            this.BarrelPoints = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.BarrelPoints;
        }

        @Override
        public void Start() {

        }
    }

    private class BouncePath implements Path {
        private Translation2d[] Bounce;

        public BouncePath() {
            this.Bounce = new Translation2d[] {
                    //TODO: getPoints for this
            };
        }

        @Override
        public Translation2d[] getWaypoints() {
            return this.Bounce;
        }

        @Override
        public void Start() {

        }
    }

}
