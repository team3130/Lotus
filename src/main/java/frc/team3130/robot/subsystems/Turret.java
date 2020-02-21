package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.vision.Limelight;

import static frc.team3130.robot.util.Utils.*;

public class Turret implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_turret;

    //Create and define all standard data types needed
    private static TurretState m_controlState = TurretState.IDLE;
    private static TurretState m_lastState;

    // Output value. This will be in various units depending on the control state
    private static double output = 0.0;
    private static double initialChassisHoldAngle = 0.0;

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

//        configPIDF(m_turret,
//                RobotMap.kTurretHoldP,
//                RobotMap.kTurretHoldI,
//                RobotMap.kTurretHoldD,
//                0.0);

        m_turret.clearStickyFaults();

        m_controlState = TurretState.IDLE; // Initialize turret state to IDLE

        //TODO: switch from practice bot ticks to comp bot ticks
        m_turret.setSelectedSensorPosition((int) (RobotMap.kTurretStartupAngle * RobotMap.kTurretPracticebotTicksPerDegree));

        m_turret.configForwardSoftLimitThreshold((int) (RobotMap.kTurretFwdLimit * RobotMap.kTurretPracticebotTicksPerDegree));
        m_turret.configReverseSoftLimitThreshold((int) (RobotMap.kTurretRevLimit * RobotMap.kTurretPracticebotTicksPerDegree));
    }

    /**
     * Manually move the turret
     *
     * @param speed Input range -1.0 to 1.0
     */
    public static void manualOp(double speed) {
        m_controlState = TurretState.MANUAL;
        output = speed;
    }

    /**
     * Request turret to begin Limelight-assisted aiming mode
     *
     * @param usePrediction Predict the where the target is using chassis odometry
     */
    public static void aim(boolean usePrediction) {
        if(usePrediction){
            m_controlState = TurretState.PREDICT;
        }else{
            m_controlState = TurretState.AIMING;
        }
    }

    /**
     * Flip the aiming state of the turret
     */
    public static void toggleAimState() {
        if (m_controlState == TurretState.AIMING || m_controlState == TurretState.HOLD || m_controlState == TurretState.PREDICT) {
            stow();
        } else if (m_controlState == TurretState.STOWED) {
            aim(true);
        } else {
            aim(false);
        }
    }

    /**
     * Request to stow the turret
     */
    public static void stow() {
        m_controlState = TurretState.STOWED;

        // Reset output to stowing position
        output = RobotMap.kTurretStowingAngle;
    }

    /**
     * Set the turret to idle mode
     */
    public static void idle() {
        m_controlState = TurretState.IDLE;
    }

    /**
     * System states
     */
    private enum TurretState {
        STOWED, // Turret stowed
        PREDICT, // Turret is using dead reckoning to predict where the target is
        AIMING, // Turret is aiming with Limelight assist
        HOLD, // Turret holding heading to target
        MANUAL, // Manual voltage control
        IDLE, // Turret idle
    }

    /**
     * Manage the Turret state outputs.
     */
    public static synchronized void writePeriodicOutputs() {
        // Determine if this state is new
        boolean isNewState = false;
        if (m_controlState != m_lastState) {
            isNewState = true;
        }

        /* Handle states */
        switch (m_controlState) {
            case STOWED:
                // Handle the stow state
                handleStowed(isNewState);
                break;

            case PREDICT:
                // Handle the dead reckon aiming state
                handlePredict(isNewState);
                break;

            case AIMING:
                // Handle the LL aiming state
                handleAiming(isNewState);
                break;

            case HOLD:
                // Handle holding turret heading state
                handleHold(isNewState);
                break;

            case MANUAL:
                // Handle manual control state
                handleManual();
                break;

            case IDLE:
                // Handle idle state
                handleIdle(isNewState);
                break;

            default:
                // Set the system to Stow in case it is moving and loses track of state.
                m_controlState = TurretState.STOWED;

        }

        // Set the last state
        m_lastState = m_controlState;

    }

    /**
     * Handle system stow state
     *
     * @param newState Is this state new?
     */
    private static void handleStowed(boolean newState) {
        if (newState) {
            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretP,
                    RobotMap.kTurretI,
                    RobotMap.kTurretD,
                    RobotMap.kTurretF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);

            // Send turret to stowed position
            setAngleMM(output);
        }
    }

    /**
     * Handle odometry-assisted aiming
     *
     * @param newState Is this state new?
     */
    private static void handlePredict(boolean newState) {
        if (newState) {
            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretP,
                    RobotMap.kTurretI,
                    RobotMap.kTurretD,
                    RobotMap.kTurretF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);

            //TODO: implement actual dead reckoning
            output = 180.0 - RobotMap.kChassisStartingPose.getRotation().getDegrees() - Navx.GetInstance().getHeading();

            setAngleMM(output);
        }

        if (isFinished()) {
            // Transition into Limelight aim state
            m_controlState = TurretState.AIMING;
        }
    }

    /**
     * Handle Limelight-assisted aiming
     *
     * @param newState Is this state new?
     */
    private static void handleAiming(boolean newState) {
        if (newState) {
            // Turn on Limelight
            Limelight.GetInstance().setLedState(true);

            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretP,
                    RobotMap.kTurretI,
                    RobotMap.kTurretD,
                    RobotMap.kTurretF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);
        }

        if (isOnTarget()) {
            m_controlState = TurretState.HOLD;
            // We are going out of Limelight aiming, turn off LEDs
            Limelight.GetInstance().setLedState(false);
        }

        if (Limelight.GetInstance().hasTrack()) {
            // TODO: Explain why is this negative
            double offset = -Limelight.GetInstance().getDegHorizontalError();
            //
            output = getAngleDegrees() + offset;
            setAngleMM(offset);
        }
    }

    /**
     * Handle system hold state
     *
     * @param newState Is this state new?
     */
    private static void handleHold(boolean newState) {
        if (newState) {
            // Config PIDF
            configPIDF(m_turret,
                    RobotMap.kTurretHoldP,
                    RobotMap.kTurretHoldI,
                    RobotMap.kTurretHoldD,
                    RobotMap.kTurretHoldF);
            configMotionMagic(m_turret, 0, 0);

            // Track initial Chassis heading after transitioning from another state
            initialChassisHoldAngle = Navx.GetInstance().getHeading();
        }

        // Set the angle of the turret while compensating for Chassis angle change TODO: use odometry
        setAngle(output - (Navx.GetInstance().getHeading() - initialChassisHoldAngle));

    }

    /**
     * Handle system manual state
     */
    private static void handleManual() {
        setOpenLoop(output);
    }

    /**
     * Handle system idle state
     *
     * @param newState Is this state new?
     */
    private static void handleIdle(boolean newState) {
        if (newState) {
            // Force-set output
            output = 0.0;

            // Force-set the motor to 0.0V
            m_turret.set(0.0);
        }
    }

    /**
     * Add onto the current angle of the turret using Motion Magic mode.
     *
     * @param angle_deg Angle to add in degrees, positive is CCW
     */
    public synchronized static void addAngleMM(double angle_deg) {
        output += angle_deg;
        setAngleMM(output);
    }

    /**
     * Set the desired angle of the turret using Motion Magic mode.
     *
     * @param angle_deg Absolute angle of the turret, in degrees
     */
    public synchronized static void setAngleMM(double angle_deg) {
        m_turret.set(ControlMode.MotionMagic, angle_deg * RobotMap.kTurretPracticebotTicksPerDegree);
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
    private synchronized static void setOpenLoop(double speed) {
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

    public static boolean isFinished() { //TODO: needs other isFinished checks for other states
        if (m_controlState != m_lastState) return false;
        if (m_controlState == TurretState.PREDICT && (getAngleError() < RobotMap.kTurretReadyToAimTolerance))
            return true;
        else return false;
    }


    /**
     * Turret is "OnTarget" if it is in position mode and its angle is within
     * {@code RobotMap.kTurretOnTargetTolerance} deadband angle to the setpoint.
     *
     * @return If the turret is aimed on target and in correct mode
     */
    public static boolean isOnTarget() {
        if (m_controlState == TurretState.AIMING || m_controlState == TurretState.HOLD) {
            return Math.abs(getAngleError()) < RobotMap.kTurretOnTargetTolerance;
        } else {
            return false;
        }
    }

    /**
     * Whether the turret is tracking the goal.
     *
     * @return
     */
    public static boolean isTracking() {
        return (m_controlState == TurretState.PREDICT || m_controlState == TurretState.AIMING || m_controlState == TurretState.HOLD);
    }

    public void calculateRPM() {
        if ((7 * 12) >= Limelight.GetInstance().getDistanceToTarget()) {
            Flywheel.setSpeed(3200);
        } else if ((26 * 12) <= Limelight.GetInstance().getDistanceToTarget()) {
            Flywheel.setSpeed(7500);
        } else {
            Flywheel.setSpeed((Math.pow(Limelight.GetInstance().getDistanceToTarget(), 5) / (1.2 * Math.pow(10, 9)) + 3900));
        }

    }

    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("Turret Angle", getAngleDegrees());
        SmartDashboard.putNumber("Turret Setpoint", getAngleSetpoint());
        SmartDashboard.putBoolean("Turret onTarget", isOnTarget());
        SmartDashboard.putBoolean("Turret isTracking", isTracking());
    }
}
