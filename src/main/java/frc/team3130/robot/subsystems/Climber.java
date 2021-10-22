package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class Climber extends SubsystemBase {

    //Create necessary objects
    private WPI_TalonSRX m_climberWinchLeft;
    private WPI_TalonSRX m_climberWinchRight;

    private Solenoid m_smallClimberPnuematic;
    private Solenoid m_bigClimberPnuematic;

    private static final Climber instance = new Climber();

    public static Climber getInstance() {
        return instance;
    }

    //Create and define all standard data types needed

    /**
     * used to be a singelton please use the public constructor/object defined in robotContainer
     */

    private Climber() {
        m_climberWinchLeft = new WPI_TalonSRX(RobotMap.CAN_CLIMBER1);
        m_climberWinchRight = new WPI_TalonSRX(RobotMap.CAN_CLIMBER2);

        m_bigClimberPnuematic = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_ACTUATOR);
        m_smallClimberPnuematic = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_DEPLOYER);

        setWinchToBrake();
    }

    public void leftWinch(double spin) {
        m_climberWinchLeft.set(spin);
    }

    public void rightWinch(double spin) {
        m_climberWinchRight.set(spin);
    }

    //method for deploying wheel to be called in a command
    public void toggleSmall() {
        m_smallClimberPnuematic.set(!m_smallClimberPnuematic.get());
    }

    //method for deploying wheel to be called in a command
    public void toggleBig() {
        m_bigClimberPnuematic.set(!m_bigClimberPnuematic.get());
    }

    public void retractClimb(){
        m_bigClimberPnuematic.set(false);
        m_smallClimberPnuematic.set(false);
    }

    public void setWinchToBrake(){
        m_climberWinchLeft.setNeutralMode(NeutralMode.Brake);
        m_climberWinchRight.setNeutralMode(NeutralMode.Brake);}
}

