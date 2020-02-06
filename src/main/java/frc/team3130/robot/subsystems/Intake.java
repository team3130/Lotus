package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

public class Intake implements Subsystem {

    //Create necessary objects
    private static WPI_VictorSPX m_intakeMotor;

    //Create and define all standard data types needed

    /**
     * The Singleton instance of this Intake. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Intake INSTANCE = new Intake();

    /**
     * Returns the Singleton instance of this ExampleSubsystem. This static method
     * should be used -- {@code ExampleSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Intake getInstance() {
        return INSTANCE;
    }

    private Intake() {
        m_intakeMotor = new WPI_VictorSPX(RobotMap.CAN_INTAKE1);

        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    public static void runIntake(double speed){
        m_intakeMotor.set(ControlMode.PercentOutput, speed);
    }

}

