package frc.team3130.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Flywheel;
import frc.team3130.robot.subsystems.Hood;
import frc.team3130.robot.subsystems.Hopper;
import frc.team3130.robot.subsystems.Turret;
import frc.team3130.robot.vision.Limelight;
import frc.team3130.robot.vision.WheelSpeedCalculations;

public class Shoot extends CommandBase {
    private final Turret m_turret;
    private final Hopper m_hopper;
    private final Flywheel m_flywheel;
    private final Hood m_hood;

    private boolean justShot;
    private boolean changedState;
    private boolean isShooting;
    private double lastIndexTime;

    public Shoot(Turret subsystemT, Hopper subsystemHop, Flywheel subsystemF, Hood subsystemHood) {
        m_turret = subsystemT;
        m_hopper = subsystemHop;
        m_flywheel = subsystemF;
        m_hood = subsystemHood;

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
        justShot = true;
        changedState = true;
        isShooting = false;
        lastIndexTime = Timer.getFPGATimestamp();

        // Tell turret to hold angle
        m_turret.hold();

        // Find the flywheel speed
        if (!Limelight.GetInstance().hasTrack()){
            m_flywheel.setSpeed(3500.0);
        }else {
            double x = Limelight.GetInstance().getDistanceToTarget();
            if (71.0 <= x) {
                //Hood.setPistons(false);
                double speed = WheelSpeedCalculations.GetInstance().getSpeed(x);
                m_flywheel.setSpeed(speed);
            } else{
                m_flywheel.setSpeed(3500);
            }
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (justShot) {
            if (changedState) {
                lastIndexTime = Timer.getFPGATimestamp();
                changedState = false;
            }
            if (m_hopper.isEmpty()) {
                lastIndexTime = Timer.getFPGATimestamp();
                m_hopper.runHopperTop(0.20);
                m_hopper.runHopperLeft(-0.5);
                m_hopper.runHopperRight(-0.6);
            } else {
                m_hopper.runHopperTop(0.0);
                m_hopper.runHopperLeft(0.0);
                m_hopper.runHopperRight(0.0);
                if (Timer.getFPGATimestamp() - lastIndexTime > RobotMap.kHopperChamberPause) {
                    justShot = false;
                    changedState = true;
                }
            }
        } else {
            if (changedState && m_flywheel.canShoot()) {
                m_hopper.runHopperTop(0.6);
                isShooting = true;
                changedState = false;
            } else if(!changedState) {
                if (isShooting) {
                    if (!m_flywheel.canShoot()) {
                        isShooting = false;
                    }
                } else {
                    m_hopper.runHopperTop(0.0);
                    justShot = true;
                    changedState = true;
                }
            }
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
