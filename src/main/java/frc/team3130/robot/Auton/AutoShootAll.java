package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3130.robot.commands.Flywheel.SetFlywheelRPM;
import frc.team3130.robot.commands.Hopper.HopperIn;
import frc.team3130.robot.commands.Turret.ToggleTurretAim;


public class AutoShootAll extends ParallelCommandGroup {
    /**
     * Creates a new AutoShootAll.
     */
    public AutoShootAll() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(

                /**new AutonStartTurretAim(), TODO: Replace*/ new AutoDelay(.3),
                new ParallelRaceGroup(new SetFlywheelRPM(), new HopperIn(), new AutoDelay(3))
        );
    }
}
