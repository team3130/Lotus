package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3130.robot.commands.Hopper.HopperOut;
import frc.team3130.robot.subsystems.*;


public class Shoot3 extends SequentialCommandGroup {
    private AutoDriveStraightToPoint driveBack;

    /**
     * Creates a new Shoot3.
     */
    public Shoot3(Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood, Chassis subsystemChassis) {
        driveBack = new AutoDriveStraightToPoint(subsystemChassis);

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(new AutoTurnTurret(-180.0, subsystemT), new AutoDelay(1.5)),
                new AutoDelay(0.4),
                new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood),
                new ParallelRaceGroup(new HopperOut(subsystemHop), new AutoDelay(1.5)),
                new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood),
                new ParallelRaceGroup(new StowTurret(subsystemT), new AutoDelay(1.5)),
                new ParallelRaceGroup(driveBack, new AutoDelay(2))
        );
    }

    @Override
    public void initialize() {

        driveBack.SetParam(
                24,
                2,
                0.5,
                false
        );

        super.initialize();
    }
}