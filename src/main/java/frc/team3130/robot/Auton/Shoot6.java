package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3130.robot.commands.Intake.IntakeIn;

public class Shoot6 extends SequentialCommandGroup {
    AutoDriveStraightToPoint driveBack20;
    AutoDriveStraightToPoint driveBack;
    AutoDelay shoot1Delay;
    AutoTurnTurret shootAim1;
    AutoTurnTurret shootAim2;
    AutoTurn intakeTurn;
    IntakeIn intake;
    AutoDriveStraightToPoint driveBackIntake;
    AutoDriveStraightToPoint driveUp;
    AutoDelay shoot2Delay;

    /**
     * Creates a new Shoot6.
     */
    public Shoot6() {
        driveBack20 = new AutoDriveStraightToPoint();
        shootAim1 = new AutoTurnTurret(-180.0);
        shoot1Delay = new AutoDelay(2);
        intakeTurn = new AutoTurn();
        intake = new IntakeIn();
        driveBackIntake = new AutoDriveStraightToPoint();
        driveUp = new AutoDriveStraightToPoint();
        driveBack = new AutoDriveStraightToPoint();
        shootAim2 = new AutoTurnTurret(-180.0);
        shoot2Delay = new AutoDelay(3);

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(shootAim1, driveBack20, new AutoDelay(2)),
                new AutoDelay(0.25),
                new AutoShootAll(),
                new ParallelRaceGroup(intakeTurn, new AutoDelay(2)),
                new ParallelDeadlineGroup(
                        new ParallelRaceGroup(driveBackIntake, new AutoDelay(4)),
                        intake
                ),
                new AutoDelay(0.5),
                new ParallelRaceGroup(driveUp, new AutoDelay(8)),
                new ParallelRaceGroup(shootAim2, driveBack, new AutoDelay(2)),
                new AutoDelay(0.25),
                new AutoShootAll()
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

        intakeTurn.setParam(
                -5, //turn angle (degrees)
                0.5,//tolerance (degrees)
                true//small angle
        );

        driveBackIntake.SetParam(
                12 * 13,
                6,
                0.35,
                true
        );

        driveUp.SetParam(
                -12 * 13,
                6,
                0.6,
                true
        );

        super.initialize();
    }
}
