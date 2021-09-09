package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

public class PneuHood extends SubsystemBase {

    private Solenoid m_hood;


    public PneuHood(){
        m_hood = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_HOODPISTONS);

    }

    public void toggleHood(){
        m_hood.set(!m_hood.get());
    }

    public void acuateHood(){
    if(m_hood.get() == false)
        m_hood.set(true);
    }

    public void retractHood(){
    if(m_hood.get()){
        m_hood.set(false);
    }


    }

    public void setHoodCalc(Double dist){
        if(dist > 45)
            this.acuateHood();
        else
            this.retractHood();
    }
}
