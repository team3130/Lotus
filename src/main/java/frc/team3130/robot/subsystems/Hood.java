package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class Hood extends SubsystemBase {

    private Solenoid m_hood;

    private static final Hood instance = new Hood();

    public static Hood getInstance() {
        return instance;
    }

    private Hood(){
        m_hood = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_HOODPISTONS);
        m_hood.set(false);
    }

    public void toggleHood(){
        m_hood.set(!m_hood.get());
    }

    public void setPiston(boolean state) {
        m_hood.set(state);
    }

    public void outputToShuffleboard(){
        SmartDashboard.putBoolean("Actuated: ", m_hood.get());
    }
}
