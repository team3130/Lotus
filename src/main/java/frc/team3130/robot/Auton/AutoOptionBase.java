package frc.team3130.robot.Auton;

public class AutoOptionBase {
    protected final double updateRate_ = 1.0 / 50.0;
    protected boolean m_isActive = false;
    protected boolean m_isInterrupted_ = false;


    public void run() {
        m_isActive = true;
    }
}
