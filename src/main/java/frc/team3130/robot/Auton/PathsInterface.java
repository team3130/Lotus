package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public interface PathsInterface {
    public Trajectory getWaypoints();
    public void Start();
}
