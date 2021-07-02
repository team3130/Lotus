package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.team3130.robot.commands.Shoot.Shoot;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

public class Chooser {
    RamseteCommand autonomousCommand = null;

    private SendableChooser<Command> chooser = new SendableChooser<>();
    String[] GalacticSearches;

    private Chassis m_chassis;

    private String[] paths = {"B1D2Markers", "B1toB8", "BarrelRacing", "Bounce", "DriveInS", "DriveStraight", "GalacticSearchABlue", "GalacticSearchARed", "GalacticSearchBBlue", "GalacticSearchBRed", "QuestionMark", "Slalom"};
    private HashMap<String, CommandsAndPoses> commands = new HashMap<>();

    // steps - 1
    private int shoot6Steps = 1;
    private int stepCount = 0;
    // number of commands before starts shooting index starts at 0
    private int shotCountFirst = 0;
    // number of commands before the next shooting index starts at 0
    private int shotCountSecond = 3;

    // both shooting and drivign
    private SequentialCommandGroup shoot6;
    // command group for the first shots
    private ParallelCommandGroup shoot6shootfirst;
    private ParallelCommandGroup shoot6shootsecond;

    private Turret turret;
    private Hood hood;
    private Flywheel flywheel;
    private Hopper hopper;

    public Chooser(Chassis m_chassis, Turret turret, Hood hood, Flywheel flywheel, Hopper hopper) {
        this.m_chassis = m_chassis;
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.hopper = hopper;

        TrajectoryConfig config = new TrajectoryConfig(3,
                3);

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
                    new RamseteController(2.0, 0.7),
                    m_chassis.getFeedforward(),
                    m_chassis.getmKinematics(),
                    m_chassis::getSpeeds,
                    m_chassis.getleftPIDController(),
                    m_chassis.getRightPIDController(),
                    m_chassis::setOutput,
                    m_chassis
            );
            command.addRequirements(m_chassis);
            command.setName(paths[looper]);
            commands.put(paths[looper], new CommandsAndPoses(trajectoryTemp.getInitialPose(), command));
        }
    }

    /**
     * It registers the sendable chooser and stuff
     */
    public void chooserRegistry() {
        // adds shoot 6 (it's all good bc it just stores the reference)
        chooser.addOption("shoot6", shoot6);
        // for loop that iterates through all the paths
        for (Map.Entry map : commands.entrySet()) {
            if (((String) map.getKey()).contains("shoot6")) {
                Command cmd = (Command) map.getValue();
                if (stepCount == shotCountFirst) {
                    shoot6shootfirst.addCommands(cmd);
                    shoot6shootfirst.addCommands(new SequentialCommandGroup(new ToggleTurretAim(turret, hood) , new WaitCommand(0.5), new Shoot(turret, hopper, flywheel, hood)));
                    cmd = shoot6shootfirst;
                }
                if (stepCount == shotCountSecond) {
                    shoot6shootsecond.addCommands(cmd);
                    shoot6shootfirst.addCommands(new SequentialCommandGroup(new ToggleTurretAim(turret, hood) , new WaitCommand(0.5), new Shoot(turret, hopper, flywheel, hood)));
                    cmd = shoot6shootsecond;
                }
                stepCount++;
                shoot6.addCommands(cmd);
            }
            chooser.addOption((String) map.getKey(), ((CommandsAndPoses) map.getValue()).getCommand());
        }
        //gives chooser to smart dashboard
        SmartDashboard.putData("Auto mode", chooser);
        // Assert.assertEquals(chooser.hashCode(), commands.hashCode());
    }

    public RamseteCommand getCommand() {
        if (chooser.getSelected() == null) {
            System.out.println("dashboard is null!");
            DriverStation.reportError("selected path was null", false);
            return commands.get("DriveStraight").getCommand();
        } else {
            autonomousCommand = (RamseteCommand) chooser.getSelected();
            System.out.println(autonomousCommand.getName() + "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        }
        return autonomousCommand;
    }

    public Pose2d getInitialPose(){
        return commands.get(getCommand().getName()).getInitPose();
    }

}