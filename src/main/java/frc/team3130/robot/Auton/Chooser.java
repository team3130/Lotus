package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import java.util.LinkedHashMap;
import java.util.Map;

public class Chooser {
    RamseteCommand autonomousCommand = null;

    private SendableChooser<Command> chooser = new SendableChooser<>();
    String[] GalacticSearches;

    private Chassis m_chassis;

    private String[] paths = {"B1D2Markers", "B1toB8", "BarrelRacing", "Bounce", "DriveInS", "DriveStraight", "GalacticSearchABlue", "GalacticSearchARed", "GalacticSearchBBlue", "GalacticSearchBRed", "QuestionMark", "Slalom"};
    private LinkedHashMap<String, RamseteCommand> commands = new LinkedHashMap<>();

    /**
     * Constructs Chassis
     * @param m_chassis Chassis subsystem
     */
    public Chooser(Chassis m_chassis) {
        this.m_chassis = m_chassis;

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(RobotMap.kMaxVelocityPerSecond)/3,
                Units.feetToMeters(RobotMap.kMaxAccelerationPerSecond)/3);

        config.setKinematics(m_chassis.getmKinematics());

        for (int looper = 0; looper != paths.length; looper++) {
            // variably call Json file
            String trajectoryJSON = "/home/lvuser/deploy/paths/" + paths[looper] + ".wpilib.json";
            Trajectory trajectoryTemp = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectoryTemp = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }

            // creating a Ramsete command which is used in AutonInit
            RamseteCommand command = new RamseteCommand(
                    trajectoryTemp,
                    m_chassis::getPose,
                    new RamseteController(2.0, 0.7), //Working
                    m_chassis.getFeedforward(),
                    m_chassis.getmKinematics(), //Working
                    m_chassis::getSpeeds,
                    m_chassis.getleftPIDController(), //Working
                    m_chassis.getRightPIDController(), //Working
                    m_chassis::setOutput, //Working
                    m_chassis
            );
            command.addRequirements(m_chassis);
            command.setName(paths[looper]);
            commands.put(paths[looper], command);
        }
    }

    /**
     *
     * @return the LinkedHashMap for our generated trajectories
     */

    public LinkedHashMap<String, RamseteCommand> getAutonomousCommands() {
        return commands;
    }

    /**
     * Generates Trajectories
     */

    public void chooserRegistry() {
        // to check in if statements if a galactic search path is being selected
        GalacticSearches = new String[]{"GalacticSearchABlue", "GalacticSearchARed", "GalacticSearchBBlue", "GalacticSearchBRed"};

        // for loop that iterates through all the paths
        for (Map.Entry map : getAutonomousCommands().entrySet()) {
            try {
                // checking if it is a blue path
                if (map.getKey().equals(GalacticSearches[0]) || map.getKey().equals(GalacticSearches[2])) {
                    // this is only here because the compiler didn't like me using one of the String methods in getKey
                    String tempStr = (String) map.getKey();
                    // adds the string GalacticSearchA or GalacticSearchB, subtracts one because length is +1 the subtracts the amount of letters in blue, then uses Drive Straight as a default path
                    chooser.addOption(tempStr.substring(0, tempStr.length() - 4), null);
                }
                else {
                    // adds every other path to chooser
                    chooser.addOption((String) map.getKey(), (RamseteCommand) map.getValue());
                }
            }
            catch (IndexOutOfBoundsException e) {
                // just in case my logic is screwy
                DriverStation.reportError("Couldn't generate all autonomous commands",  false);
            }
        }
        //gives chooser to smart dashboard
        SmartDashboard.putData("Auto mode", chooser);
    }

    /**
     * To be called on autonInit
     * @return The autonomous command to be used
     */

    public RamseteCommand getCommand() {
        if (chooser.getSelected() == null) {
            System.out.println("dashboard is null!");
            autonomousCommand = getAutonomousCommands().get("DriveStraight");
            DriverStation.reportError("selected path was null", false);
        } else {
            autonomousCommand = (RamseteCommand) chooser.getSelected();
            m_chassis.setInitPose(autonomousCommand.getName());
            System.out.println(autonomousCommand.getName() + "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        }
        return autonomousCommand;
    }
}