package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Climber implements Subsystem {

    //Create necessary objects
    private static WPI_TalonSRX m_skyWalker;
    private static WPI_TalonSRX m_climberWinchMaster;
    private static WPI_VictorSPX m_climberWinchSlave;

    private static Solenoid m_climberArm;

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
        m_skyWalker = new WPI_TalonSRX(RobotMap.CAN_SKYWALKER);
        m_climberWinchMaster = new WPI_TalonSRX(RobotMap.CAN_CLIMBER1);
        m_climberWinchSlave = new WPI_VictorSPX(RobotMap.CAN_CLIMBER2);

        m_climberArm = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_CLIMBERARM);

    }

    public static void motorSpin(double spin) {
        m_skyWalker.set(spin);
    }

    public static void flier(double spin) {
        m_climberWinchMaster.set(spin);
        m_climberWinchSlave.set(ControlMode.PercentOutput, spin);
    }

    //method for deploying wheel to be called in a command
    public static void toggleClimb() {
        m_climberArm.set(!m_climberArm.get());
    }

    //method for retracting wheel to be called in a command
    public static void retractClimb() {
        m_climberArm.set(false);
    }
}