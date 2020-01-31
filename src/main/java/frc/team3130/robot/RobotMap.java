package frc.team3130.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap<ClimberArm> {

    /**
     * Constants
     */

    //Chassis
    public static double kChassisWidth = 23.0; //Checked 3/23 Distance between the left and right middle wheels
    public static double kChassisLengthBumpers = 39.0; //Checked 3/23
    public static double kLWheelDiameter = 5.9; //Center wheel
    public static double kRWheelDiameter = 5.9; //Center wheel

    public static double kMaxDriveSpeed = 0.8;

    public static double kLChassisTicksPerInch = 4096.0 / (Math.PI * kLWheelDiameter);
    public static double kRChassisTicksPerInch = 4096.0 / (Math.PI * kRWheelDiameter);

    //Motion Profiling
    public static double kChassisMinPointsInBuffer = 5;
    public static double kChassisMPOutputDeadband = 0.01;
    public static int kChassisMPDefaultFireRate = 20;

    public static double kMPChassisP = 5.47; //Checked 3/23
    public static double kMPChassisI = 0.0; //Checked 3/23
    public static double kMPChassisD = 0.0; //Checked 3/23
    public static double kMPChassisF = 1023.0 / (92.0 * (kLChassisTicksPerInch + kRChassisTicksPerInch) / 2.0); //Checked 3/23

    public static double kMPMaxVel = 115.0; //maximum achievable velocity of the drivetrain in in/s NOTE: the actual motion profile should be generated at 80% of this
    public static double kMPMaxAcc = 60.0; ///maximum achievable acceleration of the drivetrain in in/s^2 NOTE: the actual motion profile should be generated at 80% of this

    public static double kDriveCodesPerRev = 4096;
    public static double kDistanceToEncoder = kDriveCodesPerRev / (Math.PI * 0.5 * (kLWheelDiameter + kRWheelDiameter));
    public static double kVelocityToEncoder = kDistanceToEncoder / 10.0;        // Per 100ms
    public static double kAccelerationToEncoder = kVelocityToEncoder / 10.0;    // Per 100ms

    public static double kChassisHighP = 0.02; //0.018
    public static double kChassisHighI = 0;
    public static double kChassisHighD = 0.09; //0.062

    public static double kChassisLowP = 0.03;
    public static double kChassisLowI = 0;
    public static double kChassisLowD = 0.11;

    public static double kDriveDeadband = 0.02;

    //Intake
    public static double kIntakeTriggerDeadband = 0.4;


    /**
     * CAN IDs
     */
    public static final int CAN_PNMMODULE = 1;

    public static final int CAN_RIGHTMOTORFRONT = 2;
    public static final int CAN_RIGHTMOTORREAR = 3;
    public static final int CAN_LEFTMOTORFRONT = 4;
    public static final int CAN_LEFTMOTORREAR = 5;

    public static final int CAN_WHEELOFFORTUNE = 9;

    public static final int CAN_SKYWALKER = 106; //TODO: map these two

    public static final int CAN_SHOOTERTOP = 50;
    public static final int CAN_SHOOTERRIGHT = 10;
    public static final int CAN_SHOOTERLEFT = 11;


    public static final int CAN_INTAKE1 = 101; //TODO: map these
    public static final int CAN_HOPPERL = 8;
    public static final int CAN_HOPPERR = 10;
    public static final int CAN_HOPPERTOP = 7;
    /**
     * Pneumatics ports
     */
    public static final int PNM_SHIFT = 0;
    public static final int PNM_CLIMBERARM = 1; //TODO: assign a button to this
    public static final int PNM_WHEELARM = 2; //TODO: assign a button, Maybe y with a delay before spin)


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


