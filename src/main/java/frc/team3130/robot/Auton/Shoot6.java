package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.commands.Intake.IntakeIn;
import frc.team3130.robot.subsystems.*;

public class Shoot6 extends SequentialCommandGroup {
    AutoDriveStraightToPoint driveBack20;
    AutoDriveStraightToPoint driveBack;
    AutoTurnTurret shootAim1;
    AutoTurnTurret shootAim2;
    IntakeIn intake;
    AutoDriveStraightToPoint driveBackIntake;
    AutoDriveStraightToPoint driveUp;
    AutoDelay shoot2Delay;

    /**
     * Creates a new Shoot6.
     */
    public Shoot6(Intake subsystemI, Chassis subsystemChassis, Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood) {
        driveBack20 = new AutoDriveStraightToPoint(subsystemChassis);
        shootAim1 = new AutoTurnTurret(-160.0, subsystemT);
        intake = new IntakeIn(subsystemI);
        driveBackIntake = new AutoDriveStraightToPoint(subsystemChassis);
        driveUp = new AutoDriveStraightToPoint(subsystemChassis);
        driveBack = new AutoDriveStraightToPoint(subsystemChassis);
        shootAim2 = new AutoTurnTurret(-160.0, subsystemT);
        shoot2Delay = new AutoDelay(3);

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(shootAim1, driveBack20, new AutoDelay(2)),
                new AutoDelay(0.25),
                new ParallelRaceGroup(new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood), new AutoDelay(3)),
                new ParallelRaceGroup(new IntakeIn(subsystemI), new AutoDelay(0.1)),
                new ParallelDeadlineGroup(
                        new ParallelRaceGroup(driveBackIntake, new AutoDelay(4)),
                        intake
                ),
                new AutoDelay(0.25),
                new ParallelRaceGroup(driveUp, new AutoDelay(8)),
                new ParallelRaceGroup(shootAim2, driveBack, new AutoDelay(2)),
                new AutoDelay(0.7),
                new ParallelRaceGroup(new AutoShootAll(subsystemT, subsystemHop, subsystemF, subsystemHood), new AutoDelay(3))
        );
    }

    @Override
    public void initialize() {
        driveBack20.SetParam(
                20, //Drive Distance (inches)
                2,  //Tolerance
                0.5, //PVbus speed
                true//Nothing
        );

        driveBackIntake.SetParam(
                12 * 13,
                6,
                0.4,
                true
        );

        driveUp.SetParam(
                -12 * 12,
                6,
                0.7,
                true
        );

        super.initialize();
    }
}
