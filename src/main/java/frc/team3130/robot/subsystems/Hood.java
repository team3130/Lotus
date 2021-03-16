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
import frc.team3130.robot.util.Utils;


public class Hood extends SubsystemBase {
	private static WPI_TalonSRX m_hood;
	private HoodState m_hoodControlState;

	private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

	private NetworkTableEntry hoodAngle =
			tab.add("Set angle", 0.0).getEntry();

//	private NetworkTableEntry hoodPvalue =
//			tab.add("Set Hood P value", 0.8).getEntry();
	private NetworkTableEntry hoodDvalue =
			tab.add("Set Hood D value", 168.0).getEntry();

//	private NetworkTableEntry reverseLimitSwitchOn =
//			tab.addBoolean("Hood reverse limit switch enabled", ).getEntry();



	/**
	 * Creates a new Hood.
	 */
	public Hood() {
		m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
		m_hood.configFactoryDefault();

		m_hood.setInverted(false);
		m_hood.setSensorPhase(true);

		m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		m_hood.setSelectedSensorPosition(0,0,10);

		m_hood.configForwardSoftLimitThreshold(14000);
		m_hood.configReverseSoftLimitThreshold(0);
		m_hood.configForwardSoftLimitEnable(true);
		m_hood.configReverseSoftLimitEnable(true);

		m_hood.clearStickyFaults();

		m_hood.set(ControlMode.PercentOutput, 0.0);

		Utils.configPIDF(m_hood,
				RobotMap.kHoodP,
				RobotMap.kHoodI,
				RobotMap.kHoodD,
				RobotMap.kHoodF);
		Utils.configMotionMagic(m_hood,RobotMap.kTurretMaxAcc,RobotMap.kTurretMaxVel);

		//intialization value for hoodstate. Currently arbitrary number, should change later
		m_hoodControlState = HoodState.MAX_65;
	}

	public void moveHood(double angle){
		m_hood.set(angle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run


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
		m_hood.set(ControlMode.MotionMagic, angle_deg * RobotMap.kHoodTicksPerDegree);
	}

	public void changeHoodState(double distance){
		if (distance <= 90) {m_hoodControlState = HoodState.ZONE1_66;} //green

		else if (distance <= 150){m_hoodControlState = HoodState.ZONE3_60;}//yellow

		else if (distance <= 210){m_hoodControlState = HoodState.ZONE5_54;}//blue

		else if (distance <= 270 ){m_hoodControlState = HoodState.ZONE7_48;}//red

		changeHoodAngle();
	}

	public double getSetAngle(){
		return hoodAngle.getDouble(0);
	}


	public void changeHoodAngle(){
		switch (m_hoodControlState) {
			case ZONE1_66:
				setAngle(32);
				break;
//			case ZONE2_63:
//				setAngle(63);
//				break;
			case ZONE3_60:
				setAngle(13);
				break;
//			case ZONE4_57:
//				setAngle(57);
//				break;
			case ZONE5_54:
				setAngle(8);
				break;
//			case ZONE6_51:
//				setAngle(51);
//				break;
			case ZONE7_48:
				setAngle(3);
				break;
//			case ZONE8_45:
//				setAngle(45);
//				break;
//			case MIN_25:
//				setAngle(25);
//				break;
//			case MAX_65:
//				setAngle(65);
//				break;
		}
	}



	public synchronized static double getRelativeHoodAngle(){
		return m_hood.getSelectedSensorPosition(0) / RobotMap.kHoodTicksPerDegree;
	}
	public void outputToShuffleboard(){

		SmartDashboard.putNumber("Hood Angle", getRelativeHoodAngle());
		SmartDashboard.putNumber("Hood Encoder value", m_hood.getSelectedSensorPosition());
	}




}
