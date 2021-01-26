package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.subsystems.*;

public class Shoot5 extends SequentialCommandGroup {
    AutoDriveStraightToPoint driveToBalls;
    AutoDriveStraightToPoint driveBack;
    AutoTurnTurret shootAim;
    IntakeIn intake;

    /**
     * Creates a new Shoot5.
     */
    public Shoot5(Intake subsystem, Turret subsystemT, Chassis subsystemChassis, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood) {
        driveToBalls = new AutoDriveStraightToPoint(subsystemChassis);
        intake = new IntakeIn(subsystem);
        driveBack = new AutoDriveStraightToPoint(subsystemChassis);
        shootAim = new AutoTurnTurret(-180.0 - RobotMap.kChassisStartingPose.getRotation().getDegrees(), subsystemT);

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(driveToBalls, intake, new AutoDelay(5)),
                new ParallelRaceGroup(driveBack, new AutoDelay(5)),
                new ParallelRaceGroup(shootAim, new AutoDelay(1)),
                new AutoDelay(0.5),
                new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood)
        );
    }

    @Override
    public void initialize() {
        driveToBalls.SetParam(
                195, //Drive Distance (inches)
                2,  //Tolerance
                0.5, //PVbus speed
                true//Nothing
        );

        driveBack.SetParam(
                -195,
                6,
                0.5,
                true
        );

        super.initialize();
    }
}
