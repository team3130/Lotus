package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class WinchSub extends SubsystemBase {
    private final WPI_TalonSRX m_bigMotor;
    private boolean hold;

    private static ShuffleboardTab tab = Shuffleboard.getTab("Winch");

    public NetworkTableEntry kBigMotorP = tab.add("P", .35).getEntry();
    public NetworkTableEntry kBigMotorI = tab.add("I", 0).getEntry();
    public NetworkTableEntry kBigMotorD = tab.add("D", 0).getEntry();
    public NetworkTableEntry kBigMotorF = tab.add("F", 0).getEntry();
// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public WinchSub() {
        m_bigMotor = new WPI_TalonSRX(RobotMap.CAN_BIGMOTOR);
        configMotionMagic(m_bigMotor, RobotMap.kMotorMaxAcc, RobotMap.kMotorMaxVel);
        hold = false;
    }

    public void periodic() {
        configPIDF(m_bigMotor,
                kBigMotorP.getDouble(0.35),
                kBigMotorI.getDouble(0),
                kBigMotorD.getDouble(0),
                kBigMotorF.getDouble(0));
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

    public void spin(double sped) {
        m_bigMotor.set(ControlMode.MotionMagic , sped);
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

    public synchronized double getRelativeEncoderValue(){
        return m_bigMotor.getSelectedSensorPosition();
    }

    public synchronized void setWinchPos(double value) {
        m_bigMotor.set(ControlMode.MotionMagic, value);
    }

    public synchronized void holdPosWinch() { //for init and end only. If we keep repeatedly calling this is motor will oscillate
        if(hold){
            setWinchPos(getRelativeEncoderValue());
        }

    }

    public void setWinchHold(boolean input){
        hold = input;
    }

}

