/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;


public class Hood extends SubsystemBase {
	private static WPI_TalonSRX m_hood;

	private HoodState m_hoodControlState;

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
	}

	public void moveHood(double pVBus){
		m_hood.set(pVBus);		
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Hood Position", m_hood.getSelectedSensorPosition());
	}

	public enum HoodState {
		PREDICT,
		AIMING,
		HOLD,
		MANUAL,
		SETPOINT,
		NEAR_SHOOT,
	}
}
