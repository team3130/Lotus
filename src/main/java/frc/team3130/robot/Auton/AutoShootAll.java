package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.team3130.robot.commands.Flywheel.SetFlywheelRPM;
import frc.team3130.robot.commands.Hopper.HopperIn;


public class AutoShootAll extends ParallelCommandGroup {
    /**
     * Creates a new AutoShootAll.
     */
    public AutoShootAll() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(
                new ParallelRaceGroup(new SetFlywheelRPM(), new HopperIn(), new AutoDelay(3))
        );
    }
}
