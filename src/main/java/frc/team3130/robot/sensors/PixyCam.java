package frc.team3130.robot.sensors;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team3130.robot.SupportingClasses.BalManager;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;

import java.util.ArrayList;

public class PixyCam {

    private Pixy2 m_pixy;
    private boolean m_isPixyConnected;
    private final int m_targetError = 10; //10 pixels of lenience for checking where ball is
    //pixy cam res is 315 x 207 (horizontal x vertical)

    public PixyCam(Link link) {
        try {
            m_pixy = Pixy2.createInstance(link);
            m_pixy.init();
            m_isPixyConnected = true;
        } catch (Exception ex) {
            //If connection fails log the error
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, false);
            m_isPixyConnected = false;
        }

    }

    public PixyCam(Link link, int arg) {
        try {
            m_pixy = Pixy2.createInstance(link);
            m_pixy.init(arg);
            m_isPixyConnected = true;
        } catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, false);
            m_isPixyConnected = false;
        }

    }


    public Block largestBlock(BalManager balManager) {
        balManager.addBalls(m_pixy.getCCC().getBlockCache());
    }
    public void outputToShuffleboard() {
        //SmartDashboard.putBoolean("Pixy is connected:", this.m_isPixyConnected);
        // SmartDashboard.putNumber("Pixy block x", (double) this.largestBlock().getX());
    }


    public Pixy2 getPixy(){return m_pixy;}

    public boolean getIsConnected(){return m_isPixyConnected;}

}

