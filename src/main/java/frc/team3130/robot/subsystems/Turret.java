package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.vision.Limelight;

public class Turret implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_turret;


    //Create and define all standard data types needed
    private static boolean isAiming;

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

        // Reset Talons
        m_turret.configFactoryDefault();

        m_turret.overrideLimitSwitchesEnable(false);
        m_turret.overrideSoftLimitsEnable(true);

        m_turret.setNeutralMode(NeutralMode.Brake);

        // configure Talons
        m_turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        m_turret.setInverted(true);
        m_turret.setSensorPhase(true);

        m_turret.enableVoltageCompensation(true);


        configPIDF(m_turret,
                RobotMap.kTurretP,
                RobotMap.kTurretI,
                RobotMap.kTurretD,
                0.0);

        m_turret.clearStickyFaults();

        isAiming = false;

        //TODO: switch from practice bot ticks to comp bot ticks
        m_turret.set(ControlMode.PercentOutput, 0.0); //Reset turret talon to simple percent output mode
        m_turret.setSelectedSensorPosition((int) (RobotMap.kTurretStartupAngle* RobotMap.kTurretPracticebotTicksPerDegree));

        m_turret.configForwardSoftLimitThreshold((int) (RobotMap.kTurretFwdLimit * RobotMap.kTurretPracticebotTicksPerDegree));
        m_turret.configReverseSoftLimitThreshold((int) (RobotMap.kTurretRevLimit * RobotMap.kTurretPracticebotTicksPerDegree));


    }

    /**
     * Set the desired angle of the turret (and put it into position control mode if it isn't already).
     *
     * @param angle_deg Absolute angle of the turret, in degrees
     */
    public synchronized static void setAngle(double angle_deg) {
        // In Position mode, outputValue set is in rotations of the motor
        m_turret.set(ControlMode.Position, angle_deg * RobotMap.kTurretPracticebotTicksPerDegree);
    }

    /**
     * Manually move the turret (and put it into vbus mode if it isn't already).
     *
     * @param speed Input range -1.0 to 1.0
     */
    public synchronized static void setOpenLoop(double speed) {
        m_turret.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the absolute angle of the turret in degrees
     *
     * @return Angle of the turret in degrees
     */
    public static double getAngleDegrees() {
        return m_turret.getSelectedSensorPosition() / RobotMap.kTurretPracticebotTicksPerDegree;
    }

    /**
     * Gets the latest angle setpoint of the closed loop controller
     *
     * @return Angle setpoint in degrees
     */
    public static double getAngleSetpoint() {
        return m_turret.getClosedLoopTarget() / RobotMap.kTurretPracticebotTicksPerDegree;
    }

    /**
     * Gets the current error of the turret angle
     *
     * @return Error of angle in degrees
     */
    private static double getAngleError() {
        return getAngleSetpoint() - getAngleDegrees();
    }


    /**
     * Turret is "OnTarget" if it is in position mode and its angle is within
     * {@code RobotMap.kTurretOnTargetTolerance} deadband angle to the setpoint.
     *
     * @return If the turret is aimed on target
     */
    public static boolean isOnTarget() {
        return (m_turret.getControlMode() == ControlMode.Position
                && Math.abs(getAngleError()) < RobotMap.kTurretOnTargetTolerance);
    }

    /**
     * Set the aiming state of the turret
     *
     * @param aimState true if the turret should be in Limelight-assisted aiming mode
     */
    public static void setAimState(boolean aimState) {
        isAiming = aimState;
        if (isAiming) {
            Limelight.GetInstance().setLedState(true);
        } else {
            Limelight.GetInstance().setLedState(false);
        }
    }

    public void calculateRPM() {
        double RPM = Limelight.GetInstance().getDistanceToTarget();
        if ((7 * 12) >= RPM) {
            Flywheel.setSpeed(3200);
        } else if ((26 * 12) <= RPM){
            Flywheel.setSpeed(7500);
        }
        else{
            //TODO: make distance a variable
            //Flywheel.setSpeed((Math.pow(Limelight.GetInstance().getDistanceToTarget(), 4) / (40 * Math.pow(10,5)) + 3625));
            Flywheel.setSpeed(.0824 * Math.pow((RPM), 2)  - 21.72 * RPM + 5133.2);
        }

    }


    /**
     * Flip the aiming state of the turret
     */
    public static void toggleAimState() {
        setAimState(!isTurretAiming());
    }

    /**
     * Whether the turret is in Limelight-assisted aiming mode
     *
     * @return
     */
    public static boolean isTurretAiming() {
        return isAiming;
    }

    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("Turret Angle", getAngleDegrees());
        SmartDashboard.putNumber("Turret Setpoint", getAngleSetpoint());
        SmartDashboard.putBoolean("Turret onTarget", isOnTarget());
        SmartDashboard.putBoolean("Turret isAiming", isTurretAiming());
    }

    public static void configPIDF(WPI_TalonSRX _talon, double kP, double kI, double kD, double kF) {
        _talon.config_kP(0, kP, 0);
        _talon.config_kI(0, kI, 0);
        _talon.config_kD(0, kD, 0);
        _talon.config_kF(0, kF, 0);
    }

    public static synchronized void writePeriodicOutputs() {
        if (isAiming && Limelight.GetInstance().hasTrack()) {
            // TODO: Explain why is this negative
            double offset = -Limelight.GetInstance().getDegHorizontalError();
            double turretAngle = getAngleDegrees();
            Turret.setAngle(turretAngle + offset);
        }
    }

}
