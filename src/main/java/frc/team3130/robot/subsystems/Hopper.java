package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Hopper implements Subsystem {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.
    //Create necessary objects
    private static WPI_TalonSRX m_hopperMotorL;
    private static WPI_TalonSRX m_hopperMotorR;
    private static WPI_TalonSRX m_hopperMotorTop;
    public static DigitalInput hopperLimitSwitch = new DigitalInput(0); //change these 2 channels
    public static DigitalInput shooterLimitSwitch =new DigitalInput(1);


    //Create and define all standard data types needed

    /**
     * The Singleton instance of this ExampleSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Hopper INSTANCE = new Hopper();

    /**
     * Creates a new instance of this ExampleSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Hopper() {
        m_hopperMotorL = new WPI_TalonSRX(RobotMap.CAN_HOPPERL);
        m_hopperMotorR = new WPI_TalonSRX(RobotMap.CAN_HOPPERR);
        m_hopperMotorTop = new WPI_TalonSRX(RobotMap.CAN_HOPPERTOP);

        m_hopperMotorL.configFactoryDefault();
        m_hopperMotorR.configFactoryDefault();
        m_hopperMotorTop.configFactoryDefault();

        m_hopperMotorL.setNeutralMode(NeutralMode.Brake);
        m_hopperMotorR.setNeutralMode(NeutralMode.Brake);
        m_hopperMotorTop.setNeutralMode(NeutralMode.Brake);
    }

    public static void runHopper(double speed) {
        m_hopperMotorL.set(-speed);
        m_hopperMotorR.set(speed);
        m_hopperMotorTop.set(speed);
    }
  //  runs hopper so everything runs inwards when there is only one ball left in it
    public static void runHopperOneBall (double speed) {
        m_hopperMotorL.set(speed);
        m_hopperMotorR.set(speed);
        m_hopperMotorTop.set(speed);
    }


    /**
     * Returns the Singleton instance of this ExampleSubsystem. This static method
     * should be used -- {@code ExampleSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Hopper getInstance() {
        return INSTANCE;
    }

}

