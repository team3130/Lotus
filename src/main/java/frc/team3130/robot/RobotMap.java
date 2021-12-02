package frc.team3130.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.team3130.robot.SupportingClasses.ObjDimensions;
import frc.team3130.robot.SupportingClasses.PhysicalObject;

import java.awt.*;
import java.util.HashMap;

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

    //Debug mode (whether to print stackTraces)
    public static final boolean debug = true;

    //Which Robot
    public static boolean kUseCompbot = true;

    //NavX
    public static boolean kNavxReversed = true;

    //Chassis
    public static Pose2d kChassisStartingPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)); //-49.37 for shoot5 auto TODO: Permanent solution for this

    public static final double kChassisMaxVoltage = 12.0;

    public static double kChassisWidth = Units.inchesToMeters(28.0); //FIXME: check
    public static double kChassisLengthBumpers = 39.0; //FIXME
    public static double kLWheelDiameter = Units.inchesToMeters(6); // Center wheel
    public static double kRWheelDiameter = Units.inchesToMeters(6); // Center wheel

    public static final double kMaxHighGearDriveSpeed = 0.8;
    public static final double kMaxTurnThrottle = 0.7; // Applied on top of max drive speed

    public static final double kChassisCodesPerRev = 2048;
    public static final double kLChassisTicksPerInch = 1500;
    public static final double kRChassisTicksPerInch = 1500;
    public static double kEncoderResolution = (kUseCompbot?4096 : 2048);

    public static final double kDriveDeadband = 0.02;
    public static final double kDriveMaxRampRate = 0.7; // Minimum seconds from 0 to 100

    public static double lowGearkS = (kUseCompbot ? .643 : .615);
    public static double lowGearkV = (kUseCompbot ? .0706: .0402);
    public static double LowGearkA = (kUseCompbot ? .00648 : .0117);

    public static double highGearkS = (0.819); //Low Gear .643
    public static double highGearkV = ( 0.0343); //Low Gear .0706
    public static double highGearkA = ( 0.00437 );

    //Motion Profiling
    public static final double kChassisMinPointsInBuffer = 5;
    public static final double kChassisMPOutputDeadband = 0.01;
    public static final int kChassisMPDefaultFireRate = 20;

    public static final double kChassisGearRatio = 6.25;
    public static double kChassisLowGearRatio = 6.25;
    public static double kChassisHighGearRatio = (60/17);

    public static final double kMPChassisP = 5.47;
    public static final double kMPChassisI = 0.0;
    public static final double kMPChassisD = 0.0;
    public static final double kMPChassisF = 1023.0 / (92.0 * (kLChassisTicksPerInch + kRChassisTicksPerInch) / 2.0); //Checked 3/23
    public static final double kChassisShiftWait = 0.07;

    public static double kMPMaxVel = 115.0; //maximum achievable velocity of the drivetrain in in/s NOTE: the actual motion profile should be generated at 80% of this
    public static double kMPMaxAcc = 60.0; ///maximum achievable acceleration of the drivetrain in in/s^2 NOTE: the actual motion profile should be generated at 80% of this


    public static double kDistanceToEncoder = kLChassisTicksPerInch / (Math.PI * 0.5 * (kLWheelDiameter + kRWheelDiameter));
    public static final double kVelocityToEncoder = kDistanceToEncoder / 10.0;        // Per 100ms
    public static final double kAccelerationToEncoder = kVelocityToEncoder / 10.0;    // Per 100ms
    
    public static double kMaxAccelerationPerSecond = 518 / 12; //Feet per second squared
    public static double kMaxVelocityPerSecond = 129 / 12; //Feet per second


    //Climber
    public static final double kClimberTriggerDeadband = 0.07;

    //Turret
    // Turret pitch and roll is how much the plane of the turret's rotation isn't level
    public static final double kTurretPitch = (kUseCompbot ? -0.495 : -0.875); // Drop forward in degrees
    public static final double kTurretRoll = 0; // Roll to the right in degrees

    public static final double kTurretManualDeadband = 0.15;
    public static final double kTurretManualMultipler = 0.2;

    public static final int kTurretOffTargetMax = 25;
    public static final double kTurretStowingAngle = -90.0;
    public static final double kTurretStartupAngle = -90.0;
    public static final double kTurretFwdLimit = 15.0; // Angle in degrees
    public static final double kTurretRevLimit = -315.0; // Angle in degrees

    public static final double kTurretP = 0.8;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 168.0;
    public static final double kTurretF = 0;

    public static final double kTurretMMP = 1.8;
    public static final double kTurretMMI = 0;
    public static final double kTurretMMD = 0.7;
    public static final double kTurretMMF = 0;
    public static final int kTurretMaxAcc = 4600;
    public static final int kTurretMaxVel = 3400;

    public static final double kTurretHoldP = 1.0;
    public static final double kTurretHoldI = 0;
    public static final double kTurretHoldD = 17.0;
    public static final double kTurretHoldF = 0;

    public static final double kTurretTicksPerDegree = ( (1.0 / 360.0) * 4096.0 * (204.0 / 30.0)); // Checked 1/31
    public static final double kTurretOnTargetTolerance = 0.5; // In degrees

    public static final double kTurretReadyToAimTolerance = 5.0; // In degrees

    public static final int kLimelightFilterBufferSize = 5; // Number of samples in input filtering window
    public static final double kLimelightLatencyMs = 11.0; // Image capture latency

    public static final double kLimelightPitch =  -35.648;   // Facing up is negative Checked: 2/21
    public static final double kLimelightYaw = 2;        // Aiming bias, facing left is positive
    public static final double kLimelightRoll = 0;       // If any, drooping to right is positive
    public static final double kLimelightHeight = 21.5;     // Height of camera aperture from the ground
    public static final double kLimelightLength = 9.5;    // Distance to the turret's rotation axis
    public static final double kLimelightOffset = 0;    // Side offset from the turret's plane of symmetry (left+)
    public static final double kLimelightCalibrationDist = 120.0; // Exact horizontal distance between target and lens

    //Hood
    public static final int kHoodForward = 17669;//TODO: FIND Real Number
    public static final double kHoodTicksPerDegree = (1.0 / 360.0) * 4096 * (2211.0 / 145.0); //= (kUseCompbot ? *put comp gear ratio here* : *put practice ratio here* ;)
    public static final double kAngleChange = 0;
    public static final double kHoodP = 12;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 168;
    public static final double kHoodF = 0;

    // low gear values, see auton-into master for highgear values
    public static double kS = (kUseCompbot ? .643 : .615);
    public static double kV = (kUseCompbot ? .0706: .0402);
    public static double kA = (kUseCompbot ? .00648 : .0117);

    public static double kHoodTolerance = 2; //The tolerance for error in angles on the hood for shooting

    //Flywheel
    public static final double kFlywheelMaxVoltage = 12.0;
    public static final double kFlywheelOpenRampRate = 1.0; // Minimum amount to time in seconds for Open Loop control output to ramp up

    public static final double kFlywheelP = .22;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 12;
    public static final double kFlywheelF = (.51*1023.0)/10650.0; // Checked 2/11, Optimal speed at 51% power

    public static final double kRPMChange = 0.0;

    public static final double kFlywheelTicksPerRevolution = 2048.0 * (24.0 / 60.0); // Checked 2/11
    public static final double kFlywheelRPMtoNativeUnitsScalar = RobotMap.kFlywheelTicksPerRevolution / (10.0 * 60.0);
    public static final double kFlywheelReadyTolerance = 80.0; // In RPM

    //Hopper
    public static final double kHopperMaxVoltage = 12.0;
    public static double kHopperChamberPause = 0.2;

    //Intake
    public static double kIntakeTriggerDeadband = 0.4;

    /**
     * Computer vision
     */
    // wheel locked positions
    public static final double wheelX_Pos = 0; //TODO: Find real value (metric system)
    public static final double wheelY_Pos = 0; //TODO: Find real value (metric system)
    // bar locked positions
    public static final double barX_Pos = 0; //TODO: Find real value (metric system)
    public static final double barY_Pos = 0; //TODO: Find real value (metric system)

    // making hashmap and adding items to it
    public static HashMap<PhysicalObject, ObjDimensions> dimensionsHashMap;
    static {
        dimensionsHashMap = new HashMap<>();
        dimensionsHashMap.put(PhysicalObject.ball, new ObjDimensions(5, 5, 5)); //TODO: Find real value (metric system)
        dimensionsHashMap.put(PhysicalObject.bar, new ObjDimensions(5, 5, 5)); //TODO: Find real value (metric system)
        dimensionsHashMap.put(PhysicalObject.wheel, new ObjDimensions(5, 5, 5)); //TODO: Find real value (metric system)
        // loading zone and target are flush against wall
        dimensionsHashMap.put(PhysicalObject.loadingZone, new ObjDimensions(0, 0, 0));
        dimensionsHashMap.put(PhysicalObject.target, new ObjDimensions(0, 0, 0));
    }

    /**
     * Field parameters
     */
    public static final double VISIONTARGETHEIGHT = 98.25; // Height of vision target center

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

    public static final int CAN_WHEELOFFORTUNE = 15;

    public static final int CAN_CLIMBER1 = 7;
    public static final int CAN_CLIMBER2 = 11;

    public static final int CAN_TURRETANGLE = 6;
    public static final int CAN_FLYWHEEL1 = 14;
    public static final int CAN_FLYWHEEL2 = 13;

    public static final int CAN_INTAKE = 10;

    public static final int CAN_HOPPERL = 8;
    public static final int CAN_HOPPERR = 9;
    public static final int CAN_HOPPERTOP = 12;
    public static final int CAN_HOOD = 17;
    /**
     * Pneumatics ports
     */
    public static final int PNM_SHIFT = 0;
    public static final int PNM_INTAKE = 1;
    public static final int PNM_DEPLOYER = 2;
    public static final int PNM_WHEELARM = 3;
    public static final int PNM_HOODPISTONS = 4;
    public static final int PNM_ACTUATOR = 5;


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