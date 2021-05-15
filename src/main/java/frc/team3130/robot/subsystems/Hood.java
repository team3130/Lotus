/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.util.Utils;


public class Hood extends SubsystemBase {
    private static WPI_TalonSRX m_hood;

    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

    private NetworkTableEntry hoodAngle =
            tab.add("Set angle", 0.0).getEntry();



    /**
     * Creates a new Hood.
     */
    public Hood() {
        m_hood = new WPI_TalonSRX(RobotMap.CAN_HOOD);
        m_hood.configFactoryDefault();

        m_hood.setInverted(false);
        m_hood.setSensorPhase(true);

        m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        m_hood.setSelectedSensorPosition(0, 0, 10); //The encoder position gets reset to zero on every powercyle/redeploy

        m_hood.configForwardSoftLimitThreshold(14000); //The forward limit for hood in encoder ticks
        m_hood.configReverseSoftLimitThreshold(0); //The backwards limit for hood in encoder ticks
        m_hood.configForwardSoftLimitEnable(true);
        m_hood.configReverseSoftLimitEnable(true);

        m_hood.clearStickyFaults();

        m_hood.set(ControlMode.PercentOutput, 0.0);

        Utils.configPIDF(m_hood,
                RobotMap.kHoodP,
                RobotMap.kHoodI,
                RobotMap.kHoodD,
                RobotMap.kHoodF);
        Utils.configMotionMagic(m_hood, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel); //Uses turret values for velocity and acceleration as these values are arbitrary

        //intialization value for hoodstate. Currently arbitrary number, should change later
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler ru
    }

    /**
     * @param angle_deg Angle to set the turret to in degrees
     */
    public synchronized void setAngle(double angle_deg) {
        // In Position mode, outputValue set is in rotations of the motor
        m_hood.set(ControlMode.MotionMagic, angle_deg * RobotMap.kHoodTicksPerDegree);
    }


    public double getShuffleBoardSetAngle() {
        return hoodAngle.getDouble(0);
    }

    /**
     * @return the current target angle of the hood
     */
    public double getAngleSetpoint() {
        return m_hood.getClosedLoopTarget() / RobotMap.kHoodTicksPerDegree;
    }


    /**
     * @return Whether the hood is ready to shoot
     */
    public boolean canShoot() {
        if (getAngleSetpoint() - getRelativeHoodAngle() <= 2) //Angle tolerance of 2 degrees
            return true;
        else
            return false;
    }


    /**
     * @return the current hood angle
     */
    public synchronized static double getRelativeHoodAngle() {
        return m_hood.getSelectedSensorPosition(0) / RobotMap.kHoodTicksPerDegree;
    }

    public void outputToShuffleboard() {
        SmartDashboard.putNumber("Hood Angle", getRelativeHoodAngle());
        SmartDashboard.putNumber("Hood Encoder value", m_hood.getSelectedSensorPosition());
    }


}
