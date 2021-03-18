package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class WinchSub extends SubsystemBase {
    private final WPI_TalonSRX m_bigMotor;

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public WinchSub() {
        m_bigMotor = new WPI_TalonSRX(RobotMap.CAN_BIGMOTOR);
        configPIDF(m_bigMotor,
                RobotMap.kBigMotorP,
                RobotMap.kBigMotorI,
                RobotMap.kBigMotorD,
                0.0);
        configMotionMagic(m_bigMotor, RobotMap.kMotorMaxAcc, RobotMap.kMotorMaxVel);
    }

    /**
     * Configure motion magic parameters
     * @param yoskiTalon which talon to use
     * @param acceleration maximum/target acceleration
     * @param cruiseVelocity cruise velocity
     */
    private static void configMotionMagic(WPI_TalonSRX yoskiTalon, int acceleration, int cruiseVelocity){
        yoskiTalon.configMotionCruiseVelocity(cruiseVelocity, 0);
        yoskiTalon.configMotionAcceleration(acceleration, 0);
    }

    public void spin(double rpm) {
        m_bigMotor.set(rpm);
    }
    /**
     * Configure the PIDF values of an arm motor
     * @param _talon which talon to use
     * @param kP Proportional
     * @param kI Integral
     * @param kD derivitive
     * @param kF Feed forward
     */
    public static void configPIDF(WPI_TalonSRX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }

    private enum WinchControlState{
        RAW_VBUS,
        PERCENT_OUTPUT,
        MOTION_MAGIC
    }

}

