package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Turret implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_turret;
    private static WPI_TalonFX m_flywheelMaster;
    private static WPI_TalonFX m_flywheelSlave;

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Turret. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Turret INSTANCE = new Turret();

    /**
     * Returns the Singleton instance of this Turret. This static method
     * should be used -- {@code Turret.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
        // Map CAN devices
        m_turret = new WPI_TalonSRX(RobotMap.CAN_TURRETANGLE);
        m_flywheelMaster = new WPI_TalonFX(RobotMap.CAN_FLYWHEEL1);
        m_flywheelSlave = new WPI_TalonFX(RobotMap.CAN_FLYWHEEL2);

        // Reset Talons
        m_turret.configFactoryDefault();
        m_flywheelMaster.configFactoryDefault();
        m_flywheelSlave.configFactoryDefault();

        m_turret.overrideLimitSwitchesEnable(false);
        m_turret.overrideSoftLimitsEnable(false);
        m_flywheelMaster.overrideLimitSwitchesEnable(false);
        m_flywheelSlave.overrideLimitSwitchesEnable(false);
        m_flywheelMaster.overrideSoftLimitsEnable(false);
        m_flywheelSlave.overrideSoftLimitsEnable(false);

        m_turret.setNeutralMode(NeutralMode.Brake);
        m_flywheelMaster.setNeutralMode(NeutralMode.Coast);
        m_flywheelSlave.setNeutralMode(NeutralMode.Coast);

        // configure Talons
        m_turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        m_turret.set(ControlMode.Position, 0);
        m_flywheelSlave.follow(m_flywheelMaster);

        m_turret.enableVoltageCompensation(true);
        m_flywheelMaster.enableVoltageCompensation(true);

        configPIDF(m_turret,
                RobotMap.kTurretP,
                RobotMap.kTurretI,
                RobotMap.kTurretD,
                0.0);


        m_turret.clearStickyFaults();
        m_flywheelMaster.clearStickyFaults();
        m_flywheelSlave.clearStickyFaults();

    }

    /**
     * Spin the turret flywheel at a raw percent VBus value
     *
     * @param spin percent of max voltage output
     */
    public static void spinFlywheel(double spin) {
        m_flywheelMaster.set(ControlMode.PercentOutput, spin);
    }



    // Turret Angle

    /**
     * Set the desired angle of the turret (and put it into position control mode if it isn't already).
     *
     * @param angle_deg absolute angle of the turret, in degrees
     */
    public synchronized static void setAngle(double angle_deg) {
        // In Position mode, outputValue set is in rotations of the motor
//        System.out.println("Set value:  " + (angle_deg * RobotMap.kTurretTicksPerDegree) + " -------------"); //DEBUG
        m_turret.set(ControlMode.Position, angle_deg * RobotMap.kTurretTicksPerDegree);
    }

    /**
     * Get the absolute angle of the turret in degrees
     *
     * @return angle of the turret in degrees
     */
    public static double getAngleDegrees() {
        return m_turret.getSelectedSensorPosition() / RobotMap.kTurretTicksPerDegree;
    }


    public static void outputToSmartDashboard() {
//        SmartDashboard.putNumber("turret_angle", getAngleDegrees());
//        SmartDashboard.putNumber("turret_error", getError());
//        //SmartDashboard.putNumber("turret_setpoint", getSetpoint());
//        SmartDashboard.putBoolean("turret_on_target", isOnTarget());
    }

    public static void configPIDF(WPI_TalonSRX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }
}
