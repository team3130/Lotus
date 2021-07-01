package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class CommandsAndPoses {
    private Pose2d m_initPose;
    private RamseteCommand m_command;

    public CommandsAndPoses(Pose2d initPose, RamseteCommand command) {
        m_initPose = initPose;
        m_command = command;
    }

    public RamseteCommand getCommand() {
        return m_command;
    }

    public Pose2d getInitPose() {
        return m_initPose;
    }
}
