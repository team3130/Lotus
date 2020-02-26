package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.util.Epsilon;
import frc.team3130.robot.vision.Limelight;

import static frc.team3130.robot.util.Utils.*;

public class Turret implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_turret;

    //Create and define all standard data types needed
//    private static ShuffleboardTab tab = Shuffleboard.getTab("Turret");
//    private static NetworkTableEntry testP =
//            tab.add("Turret P", 1.0)
//                    .getEntry();
//    private static NetworkTableEntry testD =
//            tab.add("Turret D", 0.0)
//                    .getEntry();

    private static TurretState m_controlState;
    private static TurretState m_lastState;

    // Output value. This will be in various units depending on the control state
    private static double output = 0.0;
    private static double initialChassisHoldAngle = 0.0;
    private static double lastHoldHeading = 0.0;
    private static int isAimedCounter = 0;

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

        m_turret.clearStickyFaults();

        m_turret.setSelectedSensorPosition((int) (RobotMap.kTurretStartupAngle * RobotMap.kTurretTicksPerDegree));

        // Set soft limits
        m_turret.configForwardSoftLimitThreshold((int) (RobotMap.kTurretFwdLimit * RobotMap.kTurretTicksPerDegree));
        m_turret.configReverseSoftLimitThreshold((int) (RobotMap.kTurretRevLimit * RobotMap.kTurretTicksPerDegree));
        m_turret.configForwardSoftLimitEnable(true);
        m_turret.configReverseSoftLimitEnable(true);

        m_controlState = TurretState.STOWED; // Initialize turret state to STOWED
        // Reset output to stowing position
        output = RobotMap.kTurretStowingAngle;
        m_lastState = TurretState.MANUAL;

        m_turret.set(ControlMode.PercentOutput, 0.0); //Reset turret talon to simple percent output mode


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
        if (usePrediction) {
            m_controlState = TurretState.PREDICT;
        } else {
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
        // Reset output to stowing position
        output = RobotMap.kTurretStowingAngle;

        m_controlState = TurretState.STOWED;
    }

    /**
     * Request to send the turret to an angle
     *
     * @param angle Frame relative angle, CCW is positive and CW is negative, 0 degrees is facing the front of the bot
     */
    public static void toAngle(double angle) {
        // Set the output to the desired turret frame-relative angle
        output = angle;

        m_controlState = TurretState.SETPOINT;
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
        SETPOINT, // Turret to angle setpoint mode
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

        /* Handle state transition */
        if (isNewState) {
            switch (m_lastState) {
                case AIMING:
                    // Handle transition out of aiming state
                    exitAiming();
                    break;

                case MANUAL:
                    // Handle transition out of aiming state
                    exitManual();
                    break;

                default:
                    // Do nothing on default
            }
        }

        // Cache current state locally
        TurretState currentState = m_controlState;

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

            case SETPOINT:
                // Handle setpoint state
                handleSetpoint(isNewState);
                break;

            case HOLD:
                // Handle holding turret heading state
                handleHold(isNewState);
                break;

            case MANUAL:
                // Handle manual control state
                handleManual(isNewState);
                break;

            default:
                // Set the system to Stow in case it is moving and loses track of state.
                m_controlState = TurretState.STOWED;

        }

        // Update the last state with the cached current state
        m_lastState = currentState;

    }

    /**
     * Handle aiming state exit
     */
    private synchronized static void exitAiming() {
    }

    /**
     * Handle manual state exit
     */
    private synchronized static void exitManual() {
        // Force-set output
        output = 0.0;

        // Force-set the motor to 0.0V
        m_turret.set(0.0);
    }


    /**
     * Handle system stow state
     *
     * @param newState Is this state new?
     */
    private synchronized static void handleStowed(boolean newState) {
        if (newState) {
            // We don't need Limelight aiming, turn off LEDs
            Limelight.GetInstance().setLedState(false);

            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretMMP,
                    RobotMap.kTurretMMI,
                    RobotMap.kTurretMMD,
                    RobotMap.kTurretMMF);
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
    private synchronized static void handlePredict(boolean newState) {
        if (newState) {
            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretMMP,
                    RobotMap.kTurretMMI,
                    RobotMap.kTurretMMD,
                    RobotMap.kTurretMMF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);
            initialChassisHoldAngle = Navx.GetInstance().getHeading();
            //TODO: implement actual dead reckoning
            output = -180.0 + RobotMap.kChassisStartingPose.getRotation().getDegrees() - initialChassisHoldAngle;

            setAngleMM(output);
        } else {
            // New setpoint if Chassis angle has changed by more that the tolerance
            if (Math.abs(Navx.GetInstance().getHeading() - initialChassisHoldAngle) > RobotMap.kTurretReadyToAimTolerance) {
                initialChassisHoldAngle = Navx.GetInstance().getHeading();
                output = -180.0 + RobotMap.kChassisStartingPose.getRotation().getDegrees() - initialChassisHoldAngle;
                setAngleMM(output);
            }
            if (isFinished()) {
                // Transition into Limelight aim state
                m_controlState = TurretState.AIMING;
            }
        }
    }

    /**
     * Handle Limelight-assisted aiming
     *
     * @param newState Is this state new?
     */
    private synchronized static void handleAiming(boolean newState) {
        if (newState) {
            // Turn on Limelight
            Limelight.GetInstance().setLedState(true);

            m_turret.set(0.0);

            // Configure PID vision
            configPIDF(m_turret,
                    RobotMap.kTurretP,
                    RobotMap.kTurretI,
                    RobotMap.kTurretD,
                    RobotMap.kTurretF);
            configMotionMagic(m_turret, 0, 0);

            // Reset aiming stability counter
            isAimedCounter = 0;
        } else {
            if (isOnTarget()) {
                isAimedCounter++; // Increment since previous periodic managed to get turret on target

                if (isAimedCounter >= RobotMap.kLimelightFilterBufferSize) { // Turret is ready to move to hold state if stable for the limelight filter window size
                    // Track initial Chassis heading before transitioning to Hold state
                    initialChassisHoldAngle = Navx.GetInstance().getHeading();

                    // Transition to Hold state
                    m_controlState = TurretState.HOLD;

                    // Break from method to immediately go to Hold state
                    return;
                }
            } else {
                // Turret got off target while settling, reset stability counter
                isAimedCounter = 0;
            }
        }
        if (Limelight.GetInstance().hasTrack()) {
            double offset = Limelight.GetInstance().getDegHorizontalError();
            output = getAngleDegrees() + offset;
            setAngle(output);
        }
    }

    /**
     * Handle system setpoint state
     *
     * @param newState Is this state new?
     */
    private synchronized static void handleSetpoint(boolean newState) {
        if (newState) {
            // We don't need Limelight aiming, turn off LEDs
            Limelight.GetInstance().setLedState(false);
            // Configure PID MM
            configPIDF(m_turret,
                    RobotMap.kTurretMMP,
                    RobotMap.kTurretMMI,
                    RobotMap.kTurretMMD,
                    RobotMap.kTurretMMF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);

            setAngleMM(output);
        } else {
            if (isFinished()) {
                // Transition into Limelight aim state
                m_controlState = TurretState.AIMING;
            }
        }
    }

    /**
     * Handle system hold state
     *
     * @param newState Is this state new?
     */
    private synchronized static void handleHold(boolean newState) {
        if (newState) {
            // Config PID hold
            configPIDF(m_turret,
                    RobotMap.kTurretHoldP,
                    RobotMap.kTurretHoldI,
                    RobotMap.kTurretHoldD,
                    RobotMap.kTurretHoldF);
            configMotionMagic(m_turret, 0, 0);
            lastHoldHeading = initialChassisHoldAngle;
        }
        double currentHeading = Navx.GetInstance().getHeading();
        if (!Epsilon.epsilonEquals(lastHoldHeading, currentHeading, 7.0)) {
            configPIDF(m_turret,
                    RobotMap.kTurretMMP,
                    RobotMap.kTurretMMI,
                    RobotMap.kTurretMMD,
                    RobotMap.kTurretMMF);
            configMotionMagic(m_turret, RobotMap.kTurretMaxAcc, RobotMap.kTurretMaxVel);
            setAngleMM(output - (currentHeading - initialChassisHoldAngle));
        } else {
            if (m_turret.getControlMode() == ControlMode.MotionMagic) {
                if (m_turret.isMotionProfileFinished()) {
                    configPIDF(m_turret,
                            RobotMap.kTurretHoldP,
                            RobotMap.kTurretHoldI,
                            RobotMap.kTurretHoldD,
                            RobotMap.kTurretHoldF);
                    configMotionMagic(m_turret, 0, 0);
                }
                // Do nothing if it's still in motion magic mode
            } else {
                // Set the angle of the turret while compensating for Chassis angle change TODO: use odometry
                setAngle(output - (currentHeading - initialChassisHoldAngle));
            }
        }

    }

    /**
     * Handle system manual state
     *
     * @param newState Is this state new?
     */
    private synchronized static void handleManual(boolean newState) {
        if (newState) {
            // We don't need Limelight aiming, turn off LEDs
            Limelight.GetInstance().setLedState(false);
        }

        setOpenLoop(output);
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
     * Set the desired angle of the turret using Motion Magic control mode.
     *
     * @param angle_deg Absolute angle of the turret, in degrees
     */
    public synchronized static void setAngleMM(double angle_deg) {
        m_turret.set(ControlMode.MotionMagic, angle_deg * RobotMap.kTurretTicksPerDegree);
    }

    /**
     * Set the desired angle of the turret using Position control mode.
     *
     * @param angle_deg Absolute angle of the turret, in degrees
     */
    public synchronized static void setAngle(double angle_deg) {
        // In Position mode, outputValue set is in rotations of the motor
        m_turret.set(ControlMode.Position, angle_deg * RobotMap.kTurretTicksPerDegree);
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
        return m_turret.getSelectedSensorPosition() / RobotMap.kTurretTicksPerDegree;
    }

    /**
     * Gets the latest angle setpoint of the closed loop controller
     *
     * @return Angle setpoint in degrees
     */
    public static double getAngleSetpoint() {
        if (m_turret.getControlMode() == ControlMode.MotionMagic || m_turret.getControlMode() == ControlMode.Position) {
            return m_turret.getClosedLoopTarget() / RobotMap.kTurretTicksPerDegree;
        }
        return 0.0;
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
        if ((m_controlState == TurretState.PREDICT || m_controlState == TurretState.SETPOINT)
                && (Math.abs(output - getAngleDegrees()) < RobotMap.kTurretReadyToAimTolerance))
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
        if ((m_controlState == TurretState.AIMING && Limelight.GetInstance().hasTrack()) || m_controlState == TurretState.HOLD) {
            return Math.abs(output - getAngleDegrees()) < RobotMap.kTurretOnTargetTolerance;
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


    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("Turret Angle", getAngleDegrees());
        SmartDashboard.putNumber("Turret Setpoint", getAngleSetpoint());
        SmartDashboard.putBoolean("Turret onTarget", isOnTarget());
        SmartDashboard.putBoolean("Turret isTracking", isTracking());
    }
}
