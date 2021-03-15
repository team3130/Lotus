package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class WinchSub extends SubsystemBase {
    private final WPI_TalonSRX m_bigMotor;

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public WinchSub() {
        m_bigMotor = new WPI_TalonSRX(RobotMap.CAN_BIGMOTOR);
    }

    public void spin(double volts) {
        m_bigMotor.setVoltage(volts);
    }

}

