package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface Path {
    public Translation2d[] getWaypoints();
    public void Start();
}
