/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.subsystems.Chassis;

public class GalacticSearchB extends CommandBase {
    private final RamseteController m_RamseteController;
    private final Chassis m_chassis;

    double delay;
    double startTime;

    /**
     * Creates a new AutoDelay.
     */
    public GalacticSearchB(double seconds, Chassis chassis) {
        m_RamseteController = new RamseteController();
        m_chassis = chassis;

        // Use addRequirements() here to declare subsystem dependencies.
        delay = seconds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > delay;
    }
}