package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class Hopper extends SubsystemBase {

    //Create necessary objects
    private static WPI_VictorSPX m_hopperMotorL;
    private static WPI_VictorSPX m_hopperMotorR;
    private static WPI_VictorSPX m_hopperMotorTop;
    private static DigitalInput m_beam;

    public Hopper() {
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

        m_hopperMotorTop.configVoltageCompSaturation(RobotMap.kHopperMaxVoltage);
        m_hopperMotorTop.enableVoltageCompensation(true);

        m_hopperMotorTop.setInverted(true);
        m_hopperMotorL.setInverted(false);
    }

    public boolean isEmpty() {
        return m_beam.get();
    }

    public void runHopperRight(double speed) {
        m_hopperMotorR.set(speed);
    }

    public void runHopperLeft(double speed) {
        m_hopperMotorL.set(speed);
    }

    public void runHopperTop(double speed) {
        m_hopperMotorTop.set(speed);
    }

    public void outputToShuffleboard() {
        SmartDashboard.putBoolean("Loader Empty", isEmpty());
    }
}

