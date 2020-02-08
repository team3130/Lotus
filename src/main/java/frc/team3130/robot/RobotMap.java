package frc.team3130.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    /**
     * Constants
     */

    //Chassis
    public static double kChassisWidth = 23.0; //FIXME
    public static double kChassisLengthBumpers = 39.0; //FIXME
    public static double kLWheelDiameter = 6.0; //Center wheel
    public static double kRWheelDiameter = 6.0; //Center wheel

    public static double kMaxDriveSpeed = 0.8;
    public static double kMaxTurnThrottle = 0.7; //Applied on top of max drive speed

    public static double kLChassisTicksPerInch = 4096.0 / (Math.PI * kLWheelDiameter); //FIXME
    public static double kRChassisTicksPerInch = 4096.0 / (Math.PI * kRWheelDiameter); //FIXME

    public static double kDriveDeadband = 0.02;

    //Motion Profiling
    public static double kChassisMinPointsInBuffer = 5;
    public static double kChassisMPOutputDeadband = 0.01;
    public static int kChassisMPDefaultFireRate = 20;

    public static double kMPChassisP = 5.47;
    public static double kMPChassisI = 0.0;
    public static double kMPChassisD = 0.0;
    public static double kMPChassisF = 1023.0 / (92.0 * (kLChassisTicksPerInch + kRChassisTicksPerInch) / 2.0); //Checked 3/23

    public static double kMPMaxVel = 115.0; //maximum achievable velocity of the drivetrain in in/s NOTE: the actual motion profile should be generated at 80% of this
    public static double kMPMaxAcc = 60.0; ///maximum achievable acceleration of the drivetrain in in/s^2 NOTE: the actual motion profile should be generated at 80% of this

    public static double kDriveCodesPerRev = 4096;
    public static double kDistanceToEncoder = kDriveCodesPerRev / (Math.PI * 0.5 * (kLWheelDiameter + kRWheelDiameter));
    public static double kVelocityToEncoder = kDistanceToEncoder / 10.0;        // Per 100ms
    public static double kAccelerationToEncoder = kVelocityToEncoder / 10.0;    // Per 100ms

    //Limelight
    public static double kLimelightTiltAngle = -19.76; // Tilting backwards is negative FIXME: calibrate
    public static double kLimelightHeight = 38.875; // Height of camera aperture from the ground FIXME
    public static double kLimelightCalibrationDist = 36.0; // Exact horizontal distance between target and lens FIXME

    //Turret
    public static double kTurretManualDeadband = 0.09;
    public static double kTurretManualMultipler = 0.15;


    public static double kTurretP = 1.4; //PID checked 2/6
    public static double kTurretI = 0;
    public static double kTurretD = 210.0;
    public static double kTurretF = 0;

    public static double kTurretTicksPerDegree = (1.0 / 360.0) * 4096.0 * (204.0 / 30.0); // checked 1/31
    public static double kTurretOnTargetTolerance = 0.5;

    //Intake
    public static double kIntakeTriggerDeadband = 0.4;

    /**
     * Field parameters
     */
    public static final double VISIONTARGETHEIGHT = 31.25; // Height of vision target center FIXME

    /**
     * Digital I/O ports
     */
    public static final int DIO_FEEDERBEAM = 0;

    /**
     * CAN IDs
     */
    public static final int CAN_PNMMODULE = 1;

    public static final int CAN_RIGHTMOTORFRONT = 2;
    public static final int CAN_RIGHTMOTORREAR = 3;
    public static final int CAN_LEFTMOTORFRONT = 4;
    public static final int CAN_LEFTMOTORREAR = 5;

    public static final int CAN_WHEELOFFORTUNE = 107; //TODO: This is wrong

    public static final int CAN_SKYWALKER = 106; //TODO: map this
    public static final int CLIMBER_ELEVATOR = 108; //TODO: map this because I just put a random number in
    public static final int CAN_CLIMBER1 = 109; //TODO: also map this because I put a random number in
    public static final int CAN_CLIMBER2 = 11;

    public static final int CAN_TURRETANGLE = 6;
    public static final int CAN_FLYWHEEL1 = 14;
    public static final int CAN_FLYWHEEL2 = 13;


    public static final int CAN_INTAKE1 = 10;

    public static final int CAN_HOPPERL = 8;
    public static final int CAN_HOPPERR = 9;
    public static final int CAN_HOPPERTOP = 12;
    /**
     * Pneumatics ports
     */
    public static final int PNM_SHIFT = 0;
    public static final int PNM_CLIMBERARM = 1;
    public static final int PNM_WHEELARM = 2;


    /**
     * Gamepad Button List
     */
    public static final int LST_BTN_A = 1;
    public static final int LST_BTN_B = 2;
    public static final int LST_BTN_X = 3;
    public static final int LST_BTN_Y = 4;
    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
    public static final int LST_BTN_WINDOW = 7;
    public static final int LST_BTN_MENU = 8;
    public static final int LST_BTN_LJOYSTICKPRESS = 9;
    public static final int LST_BTN_RJOYSTICKPRESS = 10;

    /**
     * Gamepad POV List
     */
    public static final int LST_POV_UNPRESSED = -1;
    public static final int LST_POV_N = 0;
    public static final int LST_POV_NE = 45;
    public static final int LST_POV_E = 90;
    public static final int LST_POV_SE = 135;
    public static final int LST_POV_S = 180;
    public static final int LST_POV_SW = 225;
    public static final int LST_POV_W = 270;
    public static final int LST_POV_NW = 315;


    /**
     * Gamepad Axis List
     */
    public static final int LST_AXS_LJOYSTICKX = 0;
    public static final int LST_AXS_LJOYSTICKY = 1;
    public static final int LST_AXS_LTRIGGER = 2;
    public static final int LST_AXS_RTRIGGER = 3;
    public static final int LST_AXS_RJOYSTICKX = 4;
    public static final int LST_AXS_RJOYSTICKY = 5;
}


