package frc.team3130.robot.commands.Shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.sensors.vision.HoodAngleCalculations;
import frc.team3130.robot.sensors.vision.Limelight;
import frc.team3130.robot.sensors.vision.WheelSpeedCalculations;
import frc.team3130.robot.subsystems.*;

public class Shoot extends CommandBase {
    private final Turret m_turret;
    private final Hopper m_hopper;
    private final Flywheel m_flywheel;
    private final Hood m_hood;
    private final PneuHood m_pnHood;

    private boolean justShot;
    private boolean changedState;
    private boolean isShooting;
    private boolean justStarted;
    private double lastIndexTime;
    private double pause;


    public Shoot(Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood, PneuHood subsystemP) {
        m_turret = subsystemT;
        m_hopper = subsystemHop;
        m_flywheel = subsystemF;
        m_hood = subsystemHood;
        m_pnHood = subsystemP;

        justShot = true;
        isShooting = false;
        changedState = true;
    }

    public Shoot(Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, PneuHood subsystemHood) {
        m_turret = subsystemT;
        m_hopper = subsystemHop;
        m_flywheel = subsystemF;
        m_pnHood = subsystemHood;

        justShot = true;
        isShooting = false;
        changedState = true;
    }
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Reset data trackers
        justShot = false;
        changedState = true;
        isShooting = false;
        justStarted = true;
        lastIndexTime = Timer.getFPGATimestamp();


        // Tell turret to hold angle
        m_turret.hold();

        lastIndexTime = Timer.getFPGATimestamp();

        // Find the flywheel speed
        if (!Limelight.GetInstance().hasTrack()){
            m_flywheel.setSpeed(3500.0);

            if(RobotMap.kUseVarHood){
                 m_hood.setAngle(60);
            }
            else{
                m_pnHood.acuateHood();
            }


        }else {
            double x = Limelight.GetInstance().getDistanceToTarget();
            if (5 <= x) {
                //Hood.setPistons(false);
                double speed = WheelSpeedCalculations.GetInstance().getSpeed(x);
                double angle = HoodAngleCalculations.GetInstance().getAngle(x);
                m_flywheel.setSpeed(speed);
                if(RobotMap.kUseVarHood){
                    m_hood.setAngle(angle);
                }
                else{
                    m_pnHood.setHoodCalc(angle);
                }

            } else{
                m_flywheel.setSpeed(3500);

                if(RobotMap.kUseVarHood){
                    m_hood.setAngle(60);
                }
                else{
                    m_pnHood.acuateHood();
                }
            }
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
//        if (justShot) {
//            if (changedState) {
//                lastIndexTime = Timer.getFPGATimestamp();
//                changedState = false;
//            }
//            if (m_hopper.isEmpty()) {
//                lastIndexTime = Timer.getFPGATimestamp();
//                m_hopper.runHopperTop(0.20);
//                m_hopper.runHopperLeft(-0.5);
//                m_hopper.runHopperRight(-0.6);
//            } else {
//                m_hopper.runHopperTop(0.0);
//                m_hopper.runHopperLeft(0.0);
//                m_hopper.runHopperRight(0.0);
//                if (Timer.getFPGATimestamp() - lastIndexTime > RobotMap.kHopperChamberPause) {
//                    justShot = false;
//                    changedState = true;
//                }
//            }
//        } else {
//            if (changedState && m_flywheel.canShoot()) {
//                m_hopper.runHopperTop(0.6);
//                isShooting = true;
//                changedState = false;
//            } else if(!changedState) {
//                if (isShooting) {
//                    if (!m_flywheel.canShoot()) {
//                        isShooting = false;
//                    }
//                } else {
//                    m_hopper.runHopperTop(0.0);
//                    justShot = true;
//                    changedState = true;
//                }
//            }
//        }


        System.out.println(m_hopper.getHopperTopOutput() + " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV");

        m_hopper.runHopperLeft(-0.5);
        m_hopper.runHopperRight(-0.6);
//        if(m_flywheel.canShoot() && (Timer.getFPGATimestamp() - lastIndexTime) > RobotMap.kHopperChamberPause) {
//            m_hopper.runHopperTop(.6);
//            lastIndexTime = Timer.getFPGATimestamp();
//        }
//        else
//            m_hopper.runHopperTop(0);



        if((m_flywheel.canShoot() && m_hood.canShoot()) && (Timer.getFPGATimestamp() - lastIndexTime >= .75 || justShot)){
            //TODO: figure out how to change m_hood.canShoot() without being an idiot
            justStarted = false;
            if(justShot == false){
                justShot = true;
                pause = Timer.getFPGATimestamp();
            }
            m_hopper.runHopperTop(.6);
            if(Timer.getFPGATimestamp() - pause >=  .2){
                lastIndexTime = Timer.getFPGATimestamp();
                justShot = false;
            }
        }
        else {
           m_hopper.runHopperTop(0);
        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        justShot = true;
        changedState = true;

        // Turn off hopper
        m_hopper.runHopperLeft(0.0);
        m_hopper.runHopperRight(0.0);
        m_hopper.runHopperTop(0.0);

        // Stop flywheel
        m_flywheel.stop();

        // Tell turret to aim again
        m_turret.aim(false);
    }
}
