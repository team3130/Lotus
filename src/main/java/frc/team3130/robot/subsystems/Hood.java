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

		//intialization value for hoodstate. Currently arbitrary number, should change later
		m_hoodControlState = HoodState.MAX_65;
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
		//all the angles are currently just estimated guesses. They should be refactored when we find the correct angles
		//right positive, left negative, might change later
		MIN_25, //(guessed) Minimum safe angle
		ZONE1_66, //
		ZONE2_63, //
		ZONE3_60, //
		ZONE4_57, //
		ZONE5_54, //
		ZONE6_51, //
		ZONE7_48, //
		ZONE8_45, //
		MAX_65, //(guessed) maximum safe angle
	}
}
