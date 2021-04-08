package frc.team3130.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private WPI_TalonFX m_leftMotorFront = new WPI_TalonFX(RobotMap.CAN_LEFTMOTORFRONT);
  private WPI_TalonFX m_leftMotorRear =  new WPI_TalonFX(RobotMap.CAN_LEFTMOTORREAR);
  private WPI_TalonFX m_rightMotorFront = new WPI_TalonFX(RobotMap.CAN_RIGHTMOTORFRONT);
  private WPI_TalonFX m_rightMotorRear = new WPI_TalonFX(RobotMap.CAN_RIGHTMOTORREAR);

    private ShuffleboardTab tab = Shuffleboard.getTab("Chassis");



    private NetworkTableEntry wheelMulti =
            tab.add("multi speed", 1).getEntry();


    // The motors on the left side of the drive.
    private final SpeedControllerGroup m_leftMotors =
            new SpeedControllerGroup(
                    m_leftMotorFront,
                    m_leftMotorRear);

    // The motors on the right side of the drive.
    private final SpeedControllerGroup m_rightMotors =
            new SpeedControllerGroup(
                    m_rightMotorFront,
                    m_rightMotorRear);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private TalonSRX m_turret;


    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    public DifferentialDriveKinematics getM_kinematics() {
        return m_kinematics;
    }

    private Solenoid m_shifter;


    private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(RobotMap.kChassisWidth);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        m_shifter = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_SHIFT);

        m_turret = new TalonSRX(RobotMap.CAN_TURRETANGLE);
        m_turret.setNeutralMode(NeutralMode.Brake);



        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        resetEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        if(!m_shifter.get())
        m_odometry.update(m_gyro.getRotation2d(), getDistanceLowGearL(), getDistanceLowGearR());
        else
            m_odometry.update(m_gyro.getRotation2d(), getDistanceHighGearL(), getDistanceHighGearR());

    }

    public void shift(boolean shift){
        m_shifter.set(shift);
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
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
        m_drive.feed();
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
    public double getHeading() {
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
        return ((m_leftMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kLWheelDiameter))  * 10 * wheelMulti.getDouble(1));
    }

    public double getSpeedLowGearR() {
        return (m_rightMotorFront.getSelectedSensorVelocity() / RobotMap.kEncoderResolution * (1/RobotMap.kChassisLowGearRatio) * (Math.PI * RobotMap.kLWheelDiameter)  * 10) *-1 * wheelMulti.getDouble(1);
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

      public void outputToShuffleBoard(){
    SmartDashboard.putNumber("Chassis Distance R", Units.metersToInches(getDistanceLowGearR()));
    SmartDashboard.putNumber("Chassis Distance L", Units.metersToInches(getDistanceLowGearL()));

    SmartDashboard.putNumber("Chassis Right Velocity", getSpeedLowGearR());
    SmartDashboard.putNumber("Chassis Left Velocity", getSpeedLowGearL());

    SmartDashboard.putNumber("Odemetry X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odemetry Y", m_odometry.getPoseMeters().getY());
//          System.out.println("Odemetry X: " + m_odometry.getPoseMeters().getX() );
//          System.out.println("Odemetry Y: " + m_odometry.getPoseMeters().getY() );
//          System.out.println("Heading: " + getHeading());
//
    SmartDashboard.putNumber("Heading", getHeading());

      }

}
