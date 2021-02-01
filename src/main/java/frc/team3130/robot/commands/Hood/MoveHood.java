/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.commands.Hood;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.subsystems.Hood;

public class MoveHood extends CommandBase {
	private final Hood m_hood;
	private double speed;

	private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

	private NetworkTableEntry hoodAngle =
			tab.add("angle", 1.0)
					.getEntry();

	/**
	 * Creates a new MoveHood.
	 */
	public MoveHood(double pvBus, Hood subsystem) {
		m_hood = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		m_requirements.add(m_hood);

		speed=pvBus;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_hood.moveHood(speed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_hood.moveHood(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
