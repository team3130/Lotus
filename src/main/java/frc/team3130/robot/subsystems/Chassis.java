package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.SupportingClasses.Bal;
import frc.team3130.robot.sensors.Navx;

import java.io.FileReader;

public class Chassis extends SubsystemBase {

    //Create necessary objects
    private WPI_TalonFX m_leftMotorFront;
    private WPI_TalonFX m_leftMotorRear;
    private WPI_TalonFX m_rightMotorFront;
    private WPI_TalonFX m_rightMotorRear;

    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

    private NetworkTableEntry P =
            tab.add("Chassis P", .5).getEntry();
    private NetworkTableEntry I =
            tab.add("Chassis I", 0).getEntry();
    private NetworkTableEntry D =
            tab.add("Chassis D", 0).getEntry();

    private double moveSpeed;

    private DifferentialDrive m_drive;

    private DifferentialDriveKinematics m_kinematics;
    // odometry from encoders
    // use this for angle
    private final DifferentialDriveOdometry m_odometry;

    // computer vision odometry
    // use this for robot position
    private DifferentialDriveOdometry m_CVodometry;


    private static Solenoid m_shifter;

    private SimpleMotorFeedforward m_feedforward;
    private PIDController m_leftPIDController;
    private PIDController m_rightPIDConttroller;

    private SpeedControllerGroup m_left;
    private SpeedControllerGroup m_right;


    // The gyro sensor
    private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    //Create and define all standard data types needed

    /**
     * This used to be a singelton but per the new system we are now using
     */

    /**
     * Returns the Singleton instance of this Chassis. This static method should be
     * used -- {@code Chassis.getInstance();} -- by external classes, rather than
     * the constructor to get the instance of this class.
     */

    public Chassis() {
//            super(
//                    // The ProfiledPIDController used by the subsystem
//                    new ProfiledPIDController(
//                            0,
//                            0,
//                            0,
//                            // The motion profile constraints
//                            new TrapezoidProfile.Constraints(0, 0)));

        m_leftMotorFront = new WPI_TalonFX(RobotMap.CAN_LEFTMOTORFRONT);
        m_leftMotorRear = new WPI_TalonFX(RobotMap.CAN_LEFTMOTORREAR);
        m_rightMotorFront = new WPI_TalonFX(RobotMap.CAN_RIGHTMOTORFRONT);
        m_rightMotorRear = new WPI_TalonFX(RobotMap.CAN_RIGHTMOTORREAR);

        m_leftMotorFront.configFactoryDefault();
        m_leftMotorRear.configFactoryDefault();
        m_rightMotorFront.configFactoryDefault();
        m_rightMotorRear.configFactoryDefault();


        configBrakeMode(false); // Set to coast on cstr


        /**
         * For all motors, forward is the positive direction
         *
         * Shift false is low gear
         */

        m_shifter = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_SHIFT);

        m_shifter.set(false);

        m_rightMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_leftMotorFront.configVoltageCompSaturation(RobotMap.kChassisMaxVoltage);
        m_rightMotorFront.enableVoltageCompensation(true);
        m_leftMotorFront.enableVoltageCompensation(true);


        m_rightMotorFront.setInverted(true);
        m_leftMotorFront.setInverted(false);
        m_rightMotorRear.setInverted(false);
        m_leftMotorRear.setInverted(false);

//        m_rightMotorFront.setSensorPhase(true);
//        m_leftMotorFront.setSensorPhase(true);

        m_leftMotorFront.overrideLimitSwitchesEnable(false);
        m_rightMotorFront.overrideLimitSwitchesEnable(false);

        m_left = new SpeedControllerGroup(m_leftMotorFront, m_leftMotorRear);
        m_right = new SpeedControllerGroup(m_rightMotorFront, m_rightMotorRear);

        m_drive = new DifferentialDrive(m_left, m_right);
        m_drive.setRightSideInverted(false); //Motor inversion is already handled by talon configs
        m_drive.setDeadband(RobotMap.kDriveDeadband);
        m_drive.setSafetyEnabled(false); //feed() must be called to prevent motor disable TODO: check at GF

        moveSpeed=0;

        m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));

        m_CVodometry = new DifferentialDriveOdometry(new Rotation2d(0));

        //Updated 2/2/2021 TODO tune PID values
        m_feedforward = new SimpleMotorFeedforward(RobotMap.kS,RobotMap.kV,RobotMap.kA);
        m_leftPIDController = new PIDController(2.05, 0, 0);
        m_rightPIDConttroller = new PIDController(2.05, 0, 0);

        m_leftMotorRear.follow(m_leftMotorFront);
        m_rightMotorRear.follow(m_rightMotorFront);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        Bal.m_chassis = this;
    }

    public Pose2d getPosCV() {
        return m_CVodometry.getPoseMeters();
    }

    /**
     * @param pose the starting pose of the robot in the trajectory
     * sets the initial position based off the trajectory generated
     */
    public void setInitPose(Pose2d pose) {
        m_odometry.resetPosition(pose, Navx.getRotation());
    }

    /**
     * Drive the robot using tank mode
     *
     * @param moveL         Left input throttle
     * @param moveR         Right input throttle
     * @param squaredInputs Whether or not to use squared inputs
     */
    public void driveTank(double moveL, double moveR, boolean squaredInputs) {
        //NOTE: DifferentialDrive uses set(), which sets a speed in PercentOutput mode for Talons/Victors
        m_drive.tankDrive(moveL, moveR, squaredInputs);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotorFront.setVoltage(leftVolts);
        m_rightMotorRear.setVoltage(-rightVolts);
        m_drive.feed();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        if(!m_shifter.get())
            m_odometry.update(m_gyro.getRotation2d(), getDistanceLowGearL(), getDistanceLowGearR());
        else
            m_odometry.update(m_gyro.getRotation2d(), getDistanceHighGearL(), getDistanceHighGearR());

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeedsLowGear() {
        return new DifferentialDriveWheelSpeeds((getSpeedLowGearL()/10), (getSpeedLowGearR()/10));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeedsHighGear() {
        return new DifferentialDriveWheelSpeeds((getSpeedHighGearL()/10), (getSpeedHighGearR()/10));
    }

    /**
     * Drive the robot using arcade mode
     *
     * @param moveThrottle  Base forward and backward speed to move at. Positive is
     *                      forward
     * @param turnThrottle  Turning velocity
     * @param squaredInputs Whether or not to use squared inputs
     */

    /**
     * Shifts the drivetrain gear box into an absolute gear
     *
     * @param shiftVal true is high gear, false is low gear
     */
    public void shift(boolean shiftVal) {
        m_shifter.set(shiftVal);
    }

    /**
     * Tell the Chassis to hold a relative angle
     *
     * @param angle angle to hold in degrees
     */
    public void holdAngle(double angle) {
        // TODO: Rework
    }

    /**
     * Reset the drivetrain encoder positions to 0
     */
    public void reset() {
        m_leftMotorFront.setSelectedSensorPosition(0);
        m_rightMotorFront.setSelectedSensorPosition(0);
        m_leftMotorRear.setSelectedSensorPosition(0);
        m_rightMotorRear.setSelectedSensorPosition(0);
        Navx.resetNavX();
        m_odometry.resetPosition(new Pose2d(), Navx.getRotation());
    }

    /**
     * Returns if robot is in low gear
     *
     * @return true means the robot is in low gear, false if it's in high gear
     */

    public boolean getShift() {
        return m_shifter.get();
    }

    public boolean isLowGear() {
        return !m_shifter.get();
    }

    public SimpleMotorFeedforward getFeedforward() {
        return m_feedforward;
    }

    public PIDController getleftPIDController() {
        return m_leftPIDController;
    }

    public PIDController getRightPIDController() {
        return m_rightPIDConttroller;
    }

    /**
     * Gets absolute distance traveled by the left side of the robot
     *
     * @return The absolute distance of the left side in meters
     */
    public double getDistanceL() {
        return m_leftMotorFront.getSelectedSensorPosition()/ RobotMap.kChassisCodesPerRev * (1/RobotMap.kChassisGearRatio) * ((RobotMap.kLWheelDiameter)* Math.PI);
    }

    /**
     * Gets absolute distance traveled by the right side of the robot
     *
     * @return The absolute distance of the right side in meters
     */
    public double getDistanceR() {
        return m_rightMotorFront.getSelectedSensorPosition()/ RobotMap.kChassisCodesPerRev * (1/RobotMap.kChassisGearRatio) * ((RobotMap.kRWheelDiameter)* Math.PI) * -1;
    }

    /**
     * Gets the absolute distance traveled by the robot
     *
     * @return The absolute distance traveled of robot in meters
     */
    public double getDistance() {
        return (getDistanceL() + getDistanceR()) / 2.0; //the average of the left and right distances
    }

    /**
     * Returns the current speed of the front left motor in native units
     *
     * @return Current speed of the front left motor (ticks per 0.1 seconds)
     */
    public double getRawSpeedL() {
        return m_leftMotorFront.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the current speed of the front left motor in native units
     *
     * @return Current speed of the front left motor (ticks per 0.1 seconds)
     */
    public double getRawSpeedR() {
        return m_rightMotorFront.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the current speed of the front left motor
     *
     * @return Current speed of the front left motor (meters per second)
     */
    public double getSpeedL() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return ((m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kChassisCodesPerRev * (1/RobotMap.kChassisGearRatio) * (Math.PI * RobotMap.kLWheelDiameter))  * 10);
    }

    /**
     * Returns the current speed of the front right motor
     *
     * @return Current speed of the front right motor (meters per second)
     */
    public double getSpeedR() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return ((m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kChassisCodesPerRev * (1/RobotMap.kChassisGearRatio) * (Math.PI * RobotMap.kLWheelDiameter))  * 10);
    }

    /**
     * Returns the current speed of the robot by averaging the front left and right motors
     *
     * @return Current speed of the robot
     */
    public double getSpeed() {
        return 0.5 * (getSpeedL() + getSpeedR());
    }

    /**
     * @return Raw absolute encoder ticks of the left side of the robot
     */
    public double getRawL() {
        return m_leftMotorFront.getSelectedSensorPosition(0);
    }

    /**
     * @return Raw absolute encoder ticks of the right side of the robot
     */
    public double getRawR() {
        return m_rightMotorFront.getSelectedSensorPosition(0);
    }

    /**
     * @return Returns the left master drive Talon
     */
    public WPI_TalonFX getFrontL() {
        return m_leftMotorFront;
    }

    /**
     * @return Returns the right master drive Talon
     */
    public WPI_TalonFX getFrontR() {
        return m_rightMotorFront;
    }

    //Configs

    /**
     * Set the drive talons to either Brake or Coast mode
     *
     * @param brake true for Brake mode, false for Coast mode
     */
    public void configBrakeMode(boolean brake) {
        if (brake) {
            m_leftMotorFront.setNeutralMode(NeutralMode.Brake);
            m_leftMotorRear.setNeutralMode(NeutralMode.Brake);
            m_rightMotorFront.setNeutralMode(NeutralMode.Brake);
            m_rightMotorRear.setNeutralMode(NeutralMode.Brake);
        } else {
            m_leftMotorFront.setNeutralMode(NeutralMode.Coast);
            m_leftMotorRear.setNeutralMode(NeutralMode.Coast);
            m_rightMotorFront.setNeutralMode(NeutralMode.Coast);
            m_rightMotorRear.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Configure the maximum ramping rate of the drivetrain while in Open Loop control mode
     * <p>
     * Value of 0 disables ramping
     *
     * @param maxRampRateSeconds Minimum desired time to go from neutral to full throttle
     */
    public void configRampRate(double maxRampRateSeconds) {
        m_rightMotorFront.configOpenloopRamp(maxRampRateSeconds);
        m_leftMotorFront.configOpenloopRamp(maxRampRateSeconds);
    }

//    /**
//     * Configure the drivetrain for motion profiling
//     *
//     * @param duration fire rate of the motion profile in ms
//     */
//    public void configMP(int duration) {
//
//        //left
//        m_leftMotorFront.config_kP(0, RobotMap.kMPChassisP, 0);
//        m_leftMotorFront.config_kI(0, RobotMap.kMPChassisI, 0);
//        m_leftMotorFront.config_kD(0, RobotMap.kMPChassisD, 0);
//        m_leftMotorFront.config_kF(0, RobotMap.kMPChassisF, 0);
//        m_leftMotorFront.configNeutralDeadband(RobotMap.kChassisMPOutputDeadband, 0);
//        // Status 10 provides the trajectory target for motion profile AND motion magic
//        m_leftMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, duration, 0);
//        //Profile already assumes base time is 0
//        m_leftMotorFront.configMotionProfileTrajectoryPeriod(0, 0);
//
//        //right
//        m_rightMotorFront.config_kP(0, RobotMap.kMPChassisP, 0);
//        m_rightMotorFront.config_kI(0, RobotMap.kMPChassisI, 0);
//        m_rightMotorFront.config_kD(0, RobotMap.kMPChassisD, 0);
//        m_rightMotorFront.config_kF(0, RobotMap.kMPChassisF, 0);
//        m_rightMotorFront.configNeutralDeadband(RobotMap.kChassisMPOutputDeadband, 0);
//        // Status 10 provides the trajectory target for motion profile AND motion magic
//        m_rightMotorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, duration, 0);
//        //Profile already assumes base time is 0
//        m_rightMotorFront.configMotionProfileTrajectoryPeriod(0, 0);
//    }


//    @Override
//    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
//        //Chassis ramp rate is the limit on the voltage change per cycle to prevent skidding.
//    	/*final double speedLimit = prevSpeedLimit + Preferences.getInstance().getDouble("ChassisRampRate", 0.25);
//    	if (output >  speedLimit) bias = speedLimit;
//        if (bias < -speedLimit) bias = -speedLimit;*/
//        //System.out.println("UsingTurnPID");
//        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//
//        int speed_L = (int) ((output + feedforward) * 4096);
//        int speed_R = (int) ((output + feedforward) * 4096);
//        driveTank(speed_L, speed_R, false);
//    }


    protected double getMeasurement() {
        return getAngle();
    }

    public void driveStraight(double move){
        moveSpeed=move;
    }

    public double getAngle() {
        if(Navx.getNavxPresent()){
            return Navx.getAngle(); //TODO: this CCW needs to be positive
        }else{
            //TODO:Encoder Angle, if wanted
            return 0;
        }
    }

//    /**
//     * Tell the Chassis to hold a relative angle
//     *
//     * @param angle angle to hold in degrees
//     */
//    public void holdAngle(double angle, boolean smallAngle) {
//        setPIDValues(smallAngle);
//        this.reset();
//        this.setSetpoint(getAngle()+angle);
//        this.enable();
//    }



//    public void ReleaseAngle(){
//        this.disable();
//        driveTank(0, 0, false);//Clear motors
//    }

    public DifferentialDriveKinematics getmKinematics() {
        return m_kinematics;
    }

    public void setOutput(double leftVolts, double rightVolts) {
        m_leftMotorFront.setVoltage(leftVolts);
        m_rightMotorFront.setVoltage(rightVolts);
        m_drive.feed();
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                (getSpeedL()/10),
                (getSpeedR()/10)
        );
    }

    public void setPosition(Pose2d position){
        m_odometry.resetPosition(position,Navx.getRotation());
    }

//    private void setPIDValues(boolean smallAngleTurn){//TOD2O: Tune Pid
//        if(smallAngleTurn){
//            this.getController().setPID(
//                    Preferences.getInstance().getDouble("ChassisLowP", 0.0055),
//                    Preferences.getInstance().getDouble("ChassisLowBigI", 0.015),
//                    Preferences.getInstance().getDouble("ChassisLowD", 0));
//        }else{
//            this.getController().setPID(
//                    Preferences.getInstance().getDouble("ChassisLowP", 0.0055),
//                    Preferences.getInstance().getDouble("ChassisLowI", 0.003),
//                    Preferences.getInstance().getDouble("ChassisLowD", 0));
//        }
//    }

//    public void setAbsoluteTolerance(double tolerance){
//        getController().setTolerance(tolerance);
//    }
//
//    public boolean onTarget(){
//        return getController().atSetpoint();
//    }
//
//    public TrapezoidProfile.State getSetpoint(){
//        return getController().getSetpoint();
//    }




    public void outputToShuffleboard() {
        SmartDashboard.putNumber("Chassis Right Velocity", Units.inchesToMeters(getSpeedR()));
        SmartDashboard.putNumber("Chassis Left Velocity", Units.inchesToMeters(getSpeedL()));

//        SmartDashboard.putNumber("Chassis Right Vel Traj", m_rightMotorFront.getActiveTrajectoryVelocity(0));
//        SmartDashboard.putNumber("Chassis Left Vel Traj", m_leftMotorFront.getActiveTrajectoryVelocity(0));


//        SmartDashboard.putNumber("Chassis Distance", getDistance());
//        SmartDashboard.putNumber("NavX angle", getHeading().getDegrees());

        SmartDashboard.putNumber("Chassis Distance R", getDistanceR());
        SmartDashboard.putNumber("Chassis Distance L", getDistanceL());

//        SmartDashboard.putNumber("Chassis Right Sensor Value", getRawR());
//        SmartDashboard.putNumber("Chassis Left Sensor Value", getRawL());

        SmartDashboard.putNumber("Chassis Right Output %", m_rightMotorFront.getMotorOutputPercent());
        SmartDashboard.putNumber("Chassis Left Output %", m_leftMotorFront.getMotorOutputPercent());

        SmartDashboard.putNumber("Robot position X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot position Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Robot rotation", m_odometry.getPoseMeters().getRotation().getDegrees());

    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_leftMotorFront.setSelectedSensorPosition(0);
        m_rightMotorFront.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistanceLowGear() {
        return (getDistanceLowGearL() + getDistanceLowGearR()) / 2.0;
    }





    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }




    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public static double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public double getDistanceLowGearL() {
        return m_leftMotorFront.getSelectedSensorPosition()/ RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * ((RobotMap.kLWheelDiameter)* Math.PI);
    }

    public double getDistanceLowGearR() {
        return m_rightMotorFront.getSelectedSensorPosition()/ RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * ((RobotMap.kLWheelDiameter)* Math.PI) * -1;
    }

    public double getSpeedLowGearL() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return (m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kLWheelDiameter))  * 10; //TODO: figure out what WheelMulti was from auton-re-re-work
    }

    public double getSpeedLowGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kLWheelDiameter)  * 10) * -1;
    }

    public double getDistanceHighGearL() {
        return m_leftMotorFront.getSelectedSensorPosition()/ RobotMap.kEncoderResolution * (1/RobotMap.kChassisHighGearRatio) * ((RobotMap.kLWheelDiameter)* Math.PI);
    }

    public double getDistanceHighGearR() {
        return m_rightMotorFront.getSelectedSensorPosition()/ RobotMap.kEncoderResolution * (1/RobotMap.kChassisHighGearRatio) * ((RobotMap.kLWheelDiameter)* Math.PI) * -1;
    }

    public double getSpeedHighGearL() {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return ((m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kLWheelDiameter))  * 10);
    }

    public double getSpeedHighGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisHighGearRatio) * (Math.PI * RobotMap.kLWheelDiameter)  * 10) *-1;
    }

    public void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
        //NOTE: DifferentialDrive uses set(), which sets a speed in PercentOutput mode for Talons/Victors
        m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredInputs);
    }

}
