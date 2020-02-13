package frc.team3130.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Flywheel implements Subsystem {


    //Create necessary objects
    private static WPI_TalonFX m_flywheelMaster;
    private static WPI_TalonFX m_flywheelSlave;

    //Create and define all standard data types needed

    private static ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");
    private static NetworkTableEntry testP =
            tab.add("Flywheel P", 1.0)
                    .getEntry();
    private static NetworkTableEntry testD =
            tab.add("Flywheel D", 0.0)
                    .getEntry();

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

        // configure Talons
        m_flywheelSlave.follow(m_flywheelMaster);

        m_flywheelSlave.setInverted(true);

        m_flywheelMaster.configVoltageCompSaturation(RobotMap.kFlywheelMaxVoltage);
        m_flywheelMaster.enableVoltageCompensation(true);

        m_flywheelMaster.configOpenloopRamp(RobotMap.kFlywheelOpenRampRate);

        configPIDF(m_flywheelMaster,
                RobotMap.kFlywheelP,
                RobotMap.kFlywheelI,
                RobotMap.kFlywheelD,
                RobotMap.kFlywheelF);

        m_flywheelMaster.clearStickyFaults();
        m_flywheelSlave.clearStickyFaults();

        m_flywheelMaster.set(ControlMode.PercentOutput, 0.0); //Reset flywheel master talon to simple percent output mode

    }


    /**
     * Spin the turret flywheel at a raw percent VBus value
     *
     * @param spin percent of max voltage output
     */
    public static void setOpenLoop(double spin) {
        m_flywheelMaster.set(ControlMode.PercentOutput, spin);
    }

    /**
     * Sets the RPM of the flywheel. The flywheel will then spin up to the set
     * speed.
     *
     * @param rpm flywheel RPM
     */
    public static void setSpeed(double rpm) {
        configPIDF(m_flywheelMaster, testP.getDouble(RobotMap.kFlywheelP), 0.0, testD.getDouble(RobotMap.kFlywheelD), 0.0);
        m_flywheelMaster.set(ControlMode.Velocity, Util.scaleVelocityToNativeUnits(RobotMap.kFlywheelRPMtoNativeUnitsScalar, rpm));
    }

    public static void stop() {
        setOpenLoop(0.0);
    }

    /**
     * Returns the current speed of the flywheel motor in native units
     *
     * @return Current speed of the flywheel motor (ticks per 0.1 seconds)
     */
    public static long getRawSpeed() {
        return m_flywheelMaster.getSelectedSensorVelocity(0);
    }


    public static double getRPM() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, getRawSpeed());
    }

    /**
     * Gets the latest speed setpoint of the closed loop controller
     *
     * @return speed setpoint in RPM
     */
    public static double getAngleSetpoint() {
        return Util.scaleNativeUnitsToRpm(RobotMap.kFlywheelRPMtoNativeUnitsScalar, (long) m_flywheelMaster.getClosedLoopTarget());
    }

    /**
     * Gets the current error of the flywheel speed
     *
     * @return Error of speed in rpm
     */
    private static double getAngleError() {
        return getAngleSetpoint() - getRPM();
    }

    /**
     * Gets the revolutions of the flywheel
     *
     * @return number of revolutions
     */
    public static double getRevolutions(){
        return Util.scaleNativeUnitsToRotations(RobotMap.kFlywheelTicksPerRevolution, m_flywheelMaster.getSelectedSensorPosition());
    }

    /**
     * Get the status of the flywheel if it's ready to shoot
     */
    public static boolean canShoot() {
        // Check the velocity and return true when it is within the
        // velocity target.
        if(m_flywheelMaster.getControlMode() == ControlMode.Velocity){
            return Math.abs(getAngleError()) < RobotMap.kFlywheelReadyTolerance;
        }else {
            return true;
        }
    }

    public static void outputToShuffleboard() {
        Shuffleboard.getTab("Flywheel")
                .add("Flywheel Revolutions", getRevolutions());
        Shuffleboard.getTab("Flywheel")
                .add("Flywheel RPM", getRPM());
        Shuffleboard.getTab("Flywheel")
                .add("Flywheel RPM", getRPM());
        Shuffleboard.getTab("Flywheel")
                .add("Flywheel Raw Speed", getRawSpeed());
        Shuffleboard.getTab("Flywheel")
                .add("Flywheel canShoot", canShoot());
    }

    public static void configPIDF(WPI_TalonFX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }
}

