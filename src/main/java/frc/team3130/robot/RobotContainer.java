package frc.team3130.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team3130.robot.commands.Chassis.DefaultDrive;
import frc.team3130.robot.commands.Chassis.ShiftToggle;
import frc.team3130.robot.commands.Climber.DeployBigClimber;
import frc.team3130.robot.commands.Climber.DeploySmallClimber;
import frc.team3130.robot.commands.Hood.MoveHood;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Shoot.Shoot;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.commands.Intake.IntakeOut;
import frc.team3130.robot.commands.Intake.ToggleIntake;
import frc.team3130.robot.commands.Shoot.ShootNear;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;
import frc.team3130.robot.commands.WheelOfFortune.ColorAlignment;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFLeft;
import frc.team3130.robot.commands.WheelOfFortune.SpinWOFRight;
import frc.team3130.robot.commands.WheelOfFortune.ToggleWOF;
import frc.team3130.robot.controls.JoystickTrigger;
import frc.team3130.robot.subsystems.*;


import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class RobotContainer {
    //see here for references if lost: https://github.com/wpilibsuite/allwpilib/blob/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbottraditional/RobotContainer.java

    // define Subsystems
    private final Chassis m_chassis = new Chassis();
    private final Climber m_climber = new Climber();
    private final Flywheel m_flyWheel = new Flywheel();
    private final Hood m_hood = new Hood();
    private final Hopper m_hoppper = new Hopper();
    private final Intake m_intake = new Intake();
    private final Turret m_turret = new Turret();
    private final WheelOfFortune m_wheelOfFortune = new WheelOfFortune();

    // This section is here IF we need it later
    public Chassis getChassis() {return m_chassis;}
    public Climber getClimber() {return m_climber;}
    public Flywheel getFlywheel() {return m_flyWheel;}
    public Hood getHood() {return m_hood;}
    public Hopper getHopper() {return m_hoppper;}
    public Intake getIntake() {return m_intake;}
    public Turret getTurret() {return m_turret;}
    public WheelOfFortune getWOF() {return m_wheelOfFortune;}

    private ArrayList<String> paths;
    private ArrayList<RamseteCommand> commands = new ArrayList<>();


    public static double getSkywalker() {
        double spin = 0;
        spin += m_driverGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER);
        spin -= m_driverGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER);
        return spin;
    }

    //Joysticks
    public static Joystick m_driverGamepad = new Joystick(0);
    public static Joystick m_weaponsGamepad = new Joystick(1);

    // private final Command m_BarrelRacing = new BarrelRacing(60, m_chassis);


    // Binding the buttons and triggers that are defined above to respective commands
    public RobotContainer() {
        generateTrajectories();
        configureButtonBindings();
        m_chassis.setDefaultCommand(
                new DefaultDrive(
                        m_chassis,
                        () -> m_driverGamepad.getY(GenericHID.Hand.kLeft),
                        () -> m_driverGamepad.getX(GenericHID.Hand.kRight)
                )
        );

        //TODO: complete when you have made auton commands

        /*
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
        m_chooser.addOption("Complex Auto", m_complexAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);
         */

    }

    private void configureButtonBindings() {

        /*
         * Definitions for joystick buttons start
         */

        /*
         * Drivers
         */
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_AXS_RTRIGGER).whenHeld(new IntakeIn(m_intake)); //R trigger
        new JoystickTrigger(m_driverGamepad, RobotMap.LST_AXS_LTRIGGER).whenHeld(new IntakeOut(m_intake)); // L trigger also deploys intake while active

        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RBUMPER).whenHeld(new Shoot(m_turret, m_hoppper, m_flyWheel, m_hood)); //R bumper
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LBUMPER).whenHeld(new HopperOut(m_hoppper)); //L bumper
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_RJOYSTICKPRESS).whenPressed(new ToggleTurretAim(m_turret)); //R joystick press
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_MENU).whenPressed(new ToggleIntake(m_intake));; //Menu button
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_LJOYSTICKPRESS).whenPressed(new ShiftToggle(m_chassis)); //L joystick press
        new JoystickButton(m_driverGamepad, RobotMap.LST_BTN_A).whenHeld(new ShootNear(m_turret, m_hoppper, m_flyWheel, m_hood)); //Button A


        /*
         * Weapons
         */

        new POVButton(m_driverGamepad, RobotMap.LST_POV_N).whenHeld(new MoveHood(1, m_hood));
        new POVButton(m_driverGamepad, RobotMap.LST_POV_S).whenHeld(new MoveHood(-1, m_hood));

        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_MENU).whenPressed(new DeploySmallClimber(m_climber));; //Menu button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_WINDOW).whenPressed(new DeployBigClimber(m_climber));; //Windows button
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_A).whenPressed(new ToggleWOF(m_wheelOfFortune, m_intake));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_X).whenHeld(new SpinWOFLeft(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_B).whenHeld(new SpinWOFRight(m_wheelOfFortune));
        new JoystickButton(m_weaponsGamepad, RobotMap.LST_BTN_Y).whenPressed(new ColorAlignment(m_wheelOfFortune));





    }

/*    private void setDefaultCommand() {
        //TODO: fix this I have no frickin clue what is going on here
        m_chassis.setDefaultCommand(new DefaultDrive(m_chassis, () -> driverGamepad.getY(GenericHID.Hand.kLeft), () -> driverGamepad.getX(GenericHID.Hand.kRight)));
        m_climber.setDefaultCommand(new Climber(m_climber, () -> driverGamepad.));
        m_turret.setDefaultCommand(//I DONT KNOW WHATS GOIN OOOOONNNNNNNNNNNNNNNN SETTING DEFAULT COMMANDS ARE WWWWWEEEEEIIIIRRRRDDDD );
    }*/

    public void generateTrajectories() {
        // paths = new ArrayList<>(Arrays.asList("B1D2Markers", "B1toB8", "BarrelRacing", "Bounce", "DriveInS", "DriveStraight", "GalacticSearchABlue", "GalacticSearchARed", "GalacticSearchBRed", "GalacticSearchBBlue", "QuestionMark", "Slalom"));

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(RobotMap.kMaxVelocityPerSecond)/3,
                Units.feetToMeters(RobotMap.kMaxAccelerationPerSecond)/3);

        config.setKinematics(m_chassis.getmKinematics());

        /*
        for (int looper = 0; looper != paths.size(); looper++) {
            // variably call Json file
            String trajectoryJSON = "/home/lvuser/deploy/paths/" + paths.get(looper) + ".wpilib.json";
            Trajectory trajectoryTemp = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectoryTemp = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
         */




        Trajectory trajectoryTemp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
        );

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
            commands.add(command);
            // command.setName(paths.get(looper));
        // }
    }

    public ArrayList<RamseteCommand> getAutonomousCommands() {
        return commands;
    }

    public ArrayList<String> getPaths() {
        return paths;
    }

    public void reset(){
        m_chassis.reset();
    }

}

