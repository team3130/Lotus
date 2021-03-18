package frc.team3130.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    public static final double kBigMotorP = 0.35;
    public static final double kBigMotorI = 0;
    public static final double kBigMotorD = 0;

    public static int kMotorMaxAcc = 1;
    public static int kMotorMaxVel = 1;

    public static final int CAN_BIGMOTOR = 1;

    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
}