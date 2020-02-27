package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class Shoot3 extends SequentialCommandGroup {
    private AutoDriveStraightToPoint driveBack;

    /**
     * Creates a new Shoot3.
     */
    public Shoot3() {
        driveBack = new AutoDriveStraightToPoint();

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(new AutoTurnTurret(-180.0), new AutoDelay(2)),
                new ParallelRaceGroup(new AutoShootAll(), new AutoDelay(8)),
                new ParallelRaceGroup(new StowTurret(), new AutoDelay(2)),
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