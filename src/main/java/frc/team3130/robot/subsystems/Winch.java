package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class Winch extends SubsystemBase {
    private final WPI_TalonSRX m_bigMotor;
    private boolean hold;

    private static ShuffleboardTab tab = Shuffleboard.getTab("Winch");
// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    //Create necessary objects

    //Create and define all standard data types needed
    public Winch() {
        System.out.println("WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED WINCH CREATED ");
        m_bigMotor = new WPI_TalonSRX(RobotMap.CAN_BIGMOTOR);
//        configMotionMagic(m_bigMotor, RobotMap.kMotorMaxAcc, RobotMap.kMotorMaxVel);
        hold = false;
    }


    public void spin(double sped) {
        m_bigMotor.set(sped);
        System.out.println("SPINYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY boi");
    }

}

