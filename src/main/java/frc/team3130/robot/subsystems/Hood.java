/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.sensors.vision.Limelight;


public class Hood extends SubsystemBase {
	private static WPI_TalonSRX m_hood;
	private HoodState m_hoodControlState;

	private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

	private NetworkTableEntry hoodAngle =
			tab.add("angle", 1.0)
					.getEntry();


	/**
	 * Creates a new Hood.
	 */
	public Hood() {
		m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
		m_hood.configFactoryDefault();
		m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		m_hood.setSelectedSensorPosition(0,0,10);
//		m_hood.configForwardSoftLimitThreshold(RobotMap.kHoodForward);
//		m_hood.configReverseSoftLimitThreshold(0);
		m_hood.configForwardSoftLimitEnable(false);
		m_hood.configReverseSoftLimitEnable(false);

		//intialization value for hoodstate. Currently arbitrary number, should change later
		m_hoodControlState = HoodState.MAX_65;
	}

	public void moveHood(double angle){
		m_hood.set(angle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Hood Position", m_hood.getSelectedSensorPosition());
	}


	public enum HoodState {
		//TODO: FIND Real Numbers for each position
		//right positive, left negative, might change later
		MIN_25, //(guessed) Minimum safe angle
		ZONE1_66, //Target to marker A/E3 (green)
		ZONE2_63, //
		ZONE3_60, //A/E3 to A/E5 (yellow)
		ZONE4_57, //
		ZONE5_54, // A/E5 to A/E7 (blue)
		ZONE6_51, //
		ZONE7_48, // A/E7 to A/E9 (red)
		ZONE8_45, //
		MAX_65, //(guessed) maximum safe angle
	}

	public synchronized void setAngle(double angle_deg) {
		// In Position mode, outputValue set is in rotations of the motor
		m_hood.set(ControlMode.Position, angle_deg * RobotMap.kHoodTicksPerDegree);
	}

	public void changeHoodState(double distance){
		if (distance <= 90) {m_hoodControlState = HoodState.ZONE1_66;} //green

		else if (distance <= 150){m_hoodControlState = HoodState.ZONE3_60;}//yellow

		else if (distance <= 210){m_hoodControlState = HoodState.ZONE5_54;}//blue

		else if (distance <= 270 ){m_hoodControlState = HoodState.ZONE7_48;}//red
	}

	public void changeHoodAngle(){
		switch (m_hoodControlState) {
			case ZONE1_66:
				setAngle(5);
			case ZONE2_63:
				setAngle(63);
			case ZONE3_60:
				setAngle(60);
			case ZONE4_57:
				setAngle(57);
			case ZONE5_54:
				setAngle(54);
			case ZONE6_51:
				setAngle(51);
			case ZONE7_48:
				setAngle(48);
			case ZONE8_45:
				setAngle(45);
			case MIN_25:
				setAngle(25);
			case MAX_65:
				setAngle(65);
		}
	}



	public synchronized static double getRelativeHoodAngle(){
		return m_hood.getSelectedSensorPosition(0) / RobotMap.kHoodTicksPerDegree;
	}
	public void outputToShuffleboard(){
		SmartDashboard.putNumber("Hood Angle", getRelativeHoodAngle());
	}




}
