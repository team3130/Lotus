package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Hopper implements Subsystem {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects
    private static WPI_VictorSPX m_hopperMotorL;
    private static WPI_VictorSPX m_hopperMotorR;
    private static WPI_VictorSPX m_hopperMotorTop;
    private static DigitalInput m_beam;

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Hopper. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Hopper INSTANCE = new Hopper();

    /**
     * Returns the Singleton instance of this Hopper. This static method
     * should be used -- {@code Hopper.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Hopper getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Hopper.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Hopper() {
        m_beam = new DigitalInput(RobotMap.DIO_FEEDERBEAM);
        m_hopperMotorL = new WPI_VictorSPX(RobotMap.CAN_HOPPERL);
        m_hopperMotorR = new WPI_VictorSPX(RobotMap.CAN_HOPPERR);
        m_hopperMotorTop = new WPI_VictorSPX(RobotMap.CAN_HOPPERTOP);

        m_hopperMotorL.configFactoryDefault();
        m_hopperMotorR.configFactoryDefault();
        m_hopperMotorTop.configFactoryDefault();

        m_hopperMotorL.setNeutralMode(NeutralMode.Brake);
        m_hopperMotorR.setNeutralMode(NeutralMode.Brake);
        m_hopperMotorTop.setNeutralMode(NeutralMode.Brake);

        m_hopperMotorTop.setInverted(true);
        m_hopperMotorL.setInverted(false);
    }

    public static boolean isEmpty() {
        return m_beam.get();
    }

    public static void runHopperRight(double speed) {
        m_hopperMotorR.set(speed);
    }

    public static void runHopperLeft(double speed) {
        m_hopperMotorL.set(speed);
    }

    public static void runHopperTop(double speed) {
        m_hopperMotorTop.set(speed);
    }

    public static void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Loader Empty", isEmpty());
    }
}

