/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;


public class Hood extends SubsystemBase {
	private static WPI_TalonSRX m_hood;
	private static Solenoid m_hoodPistons;

	/**
	 * Creates a new Hood.
	 */
	public Hood() {
		m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
		m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		m_hood.configForwardSoftLimitThreshold(RobotMap.kHoodForward);
		m_hood.configReverseSoftLimitThreshold(0);
		m_hood.configForwardSoftLimitEnable(true);
		m_hood.configReverseSoftLimitEnable(true);

		m_hoodPistons = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_HOODPISTONS);
		m_hoodPistons.set(false);
	}

	public void moveHood(double pVBus){
		m_hood.set(pVBus);		
	}

	public static void setPistons(boolean state){
		m_hoodPistons.set(state);
	}

	public static void toggleHoodPistons() {
		m_hoodPistons.set(!m_hoodPistons.get());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Hood Position", m_hood.getSelectedSensorPosition());
	}
}
