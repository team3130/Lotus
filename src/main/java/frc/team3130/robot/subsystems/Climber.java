package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Climber implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_climberWinchLeft;
    private static WPI_VictorSPX m_climberWinchRight;

    private static Solenoid m_smallClimberPnuematic;
    private static Solenoid m_bigClimberPnuematic;

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Climber. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Climber INSTANCE = new Climber();

    /**
     * Returns the Singleton instance of this Climber. This static method
     * should be used -- {@code Climber.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Climber getInstance() {
        return INSTANCE;
    }

    private Climber() {
        // m_skyWalker = new WPI_TalonSRX(RobotMap.CAN_SKYWALKER);
        m_climberWinchLeft = new WPI_TalonSRX(RobotMap.CAN_CLIMBER1);
        m_climberWinchRight = new WPI_VictorSPX(RobotMap.CAN_CLIMBER2);

        m_bigClimberPnuematic = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_ACTUATOR);
        m_smallClimberPnuematic = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_DEPLOYER);

        setWinchToBreak();
    }

    public static void leftWinch(double spin) {
        m_climberWinchLeft.set(spin);
    }

    public static void rightWinch(double spin) {
        m_climberWinchRight.set(ControlMode.PercentOutput, spin);
    }

    //method for deploying wheel to be called in a command
    public static void toggleSmall() {
        m_smallClimberPnuematic.set(!m_smallClimberPnuematic.get());
    }

    //method for deploying wheel to be called in a command
    public static void toggleBig() {
        m_bigClimberPnuematic.set(!m_bigClimberPnuematic.get());
    }

    public static void retractClimb(){
        m_bigClimberPnuematic.set(false);
        m_smallClimberPnuematic.set(false);
    }

    public static void setWinchToBreak(){
        m_climberWinchLeft.setNeutralMode(NeutralMode.Brake);
        m_climberWinchRight.setNeutralMode(NeutralMode.Brake);}
}

