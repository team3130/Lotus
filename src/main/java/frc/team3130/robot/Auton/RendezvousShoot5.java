package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3130.robot.Robot;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.subsystems.*;


public class RendezvousShoot5 extends SequentialCommandGroup {
    AutoDriveStraightToPoint driveToBalls;
    AutoDriveStraightToPoint driveBack;
    IntakeIn intake;


    /**
     * Creates a new Shoot3.
     */
    public RendezvousShoot5(Intake subsystem, Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood, Chassis subsystemChassis) {
        driveToBalls = new AutoDriveStraightToPoint(subsystemChassis);
        driveBack = new AutoDriveStraightToPoint(subsystemChassis);
        intake = new IntakeIn(subsystem);

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(new AutoTurnTurret(-200.0, subsystemT), new AutoDelay(1.5)),
                new AutoDelay(0.4),
                new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood),
                new ParallelRaceGroup(driveToBalls, intake, new AutoDelay(4)),
                new ParallelRaceGroup(driveBack, new AutoDelay(4)),
                new ParallelRaceGroup(new AutoTurnTurret(-200.0, subsystemT), new AutoDelay(1.5)),
                new AutoDelay(0.4),
                new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood),
                new ParallelRaceGroup(new StowTurret(subsystemT), new AutoDelay(1.5))
        );
    }

    @Override
    public void initialize() {
        driveToBalls.SetParam(
                80, //Drive Distance (inches)
                2,  //Tolerance
                0.35, //PVbus speed
                true//Nothing
        );

        driveBack.SetParam(
                -70,
                6,
                0.5,
                true
        );

        super.initialize();
    }
}