package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.team3130.robot.commands.Shoot;
import frc.team3130.robot.subsystems.Flywheel;
import frc.team3130.robot.subsystems.Hood;
import frc.team3130.robot.subsystems.Hopper;
import frc.team3130.robot.subsystems.Turret;


public class AutoShootAll extends ParallelCommandGroup {
    /**
     * Creates a new AutoShootAll.
     */
    public AutoShootAll(Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood) {

        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(
                new ParallelRaceGroup(new Shoot(subsystemT, subsystemHop, subsystemF, subsystemHood), new AutoDelay(3))
        );
    }
}
