package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class Intake extends SubsystemBase {

    //Create necessary objects
    private static WPI_VictorSPX m_intakeMotor;

    private static Solenoid m_intakeSolenoid;

    //Create and define all standard data types needed

    public Intake() {
        m_intakeMotor = new WPI_VictorSPX(RobotMap.CAN_INTAKE);

        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setNeutralMode(NeutralMode.Coast);
        m_intakeMotor.overrideLimitSwitchesEnable(false);

        m_intakeSolenoid = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_INTAKE);
    }

    public void runIntake(double speed) {
        m_intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void deployIntake() {
        m_intakeSolenoid.set(true);
    }

    public void retractIntake() {
        m_intakeSolenoid.set(false);
    }

    public void toggleIntake() {
        m_intakeSolenoid.set(!m_intakeSolenoid.get());
    }
}

