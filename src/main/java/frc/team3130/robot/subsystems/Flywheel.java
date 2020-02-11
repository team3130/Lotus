package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Flywheel implements Subsystem {


    //Create necessary objects
    private static WPI_TalonFX m_flywheelMaster;
    private static WPI_TalonFX m_flywheelSlave;

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Flywheel. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Flywheel INSTANCE = new Flywheel();

    /**
     * Returns the Singleton instance of this Flywheel. This static method
     * should be used -- {@code Flywheel.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Flywheel getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Flywheel.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Flywheel() {
        // Map CAN devices
        m_flywheelMaster = new WPI_TalonFX(RobotMap.CAN_FLYWHEEL1);
        m_flywheelSlave = new WPI_TalonFX(RobotMap.CAN_FLYWHEEL2);

        // Reset Talons
        m_flywheelMaster.configFactoryDefault();
        m_flywheelSlave.configFactoryDefault();

        m_flywheelMaster.overrideLimitSwitchesEnable(false);
        m_flywheelMaster.overrideSoftLimitsEnable(false);
        m_flywheelSlave.overrideLimitSwitchesEnable(false);
        m_flywheelSlave.overrideSoftLimitsEnable(false);

        m_flywheelMaster.setNeutralMode(NeutralMode.Coast);
        m_flywheelSlave.setNeutralMode(NeutralMode.Coast);

        m_flywheelMaster.set(ControlMode.PercentOutput, 0.0); //Reset flywheel master talon to simple percent output mode

        // configure Talons
        m_flywheelMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_flywheelSlave.follow(m_flywheelMaster);


        m_flywheelSlave.setInverted(true);

        m_flywheelMaster.enableVoltageCompensation(true);
        m_flywheelSlave.enableVoltageCompensation(true);

        configPIDF(m_flywheelMaster,
                RobotMap.kFlywheelP,
                RobotMap.kFlywheelI,
                RobotMap.kFlywheelD,
                0.0);

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


    /**
     * Returns the current speed of the flywheel motor in native units
     *
     * @return Current speed of the flywheel motor (ticks per 0.1 seconds)
     */
    public static double getRawSpeed() {
        return m_flywheelMaster.getSelectedSensorVelocity(0);
    }


    public static double getRPM() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return 10.0 * getRawSpeed() / RobotMap.kFlywheelTicksPerRevolution;
    }


    /**
     * Get the status of the flywheel if it's ready to shoot
     */
    public static boolean canShoot() {
        // Check the velocity and return true when it is within the
        // velocity target. TODO: Bogus for now. Change the comment later. TBD
        return true;
    }

    public static void outputToSmartDashboard() {
        SmartDashboard.putNumber("Flywheel RPM", getRPM());
        SmartDashboard.putNumber("Flywheel Raw Speed", getRawSpeed());
        SmartDashboard.putBoolean("Flywheel canShoot", canShoot());
    }

    public static void configPIDF(WPI_TalonFX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }
}

