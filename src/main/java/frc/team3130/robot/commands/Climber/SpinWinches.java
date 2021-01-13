package frc.team3130.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3130.robot.RobotContainer;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Climber;

public class SpinWinches extends CommandBase {
    private final Climber m_climber;

    public SpinWinches(Climber subsystem) {
        m_climber = subsystem;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {



    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {

        double spinSpeedLeft = RobotContainer.m_weaponsGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER);
        double spinSpeedRight = RobotContainer.m_weaponsGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER);

        if (spinSpeedLeft >= RobotMap.kClimberTriggerDeadband){
            m_climber.leftWinch(spinSpeedLeft);
        }else{
            m_climber.leftWinch(0);
        }

        if (spinSpeedRight >= RobotMap.kClimberTriggerDeadband){
            m_climber.rightWinch(-spinSpeedRight);
        }else{
            m_climber.rightWinch(0);
        }

        /**
        if(OI.weaponsGamepad.getRawButtonPressed(RobotMap.LST_BTN_RBUMPER)){
            Climber.rightWinch(-0.8);

        }
        if(OI.weaponsGamepad.getRawButtonPressed(RobotMap.LST_BTN_LBUMPER)) {
            Climber.leftWinch(0.8);

        }

        if(OI.weaponsGamepad.getRawButtonReleased(RobotMap.LST_BTN_RBUMPER)){
            Climber.rightWinch(0);
        }

        if (OI.weaponsGamepad.getRawButtonReleased(RobotMap.LST_BTN_LBUMPER)) {
            Climber.leftWinch(0);
        }
        */

/**

        if(OI.weaponsGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER) >= .001){
            Climber.rightWinch(-0.3);
        }
        /**else if(OI.weaponsGamepad.getRawAxis(RobotMap.LST_AXS_RTRIGGER) < .001) {
            Climber.rightWinch(0);
        }
         */
/**
        if(OI.weaponsGamepad.getRawAxis(RobotMap.LST_BTN_LBUMPER) >= .001){
            Climber.leftWinch(-0.3);
        } /**else if(OI.weaponsGamepad.getRawAxis(RobotMap.LST_AXS_LTRIGGER) < .001) {
            Climber.rightWinch(0);
        }
         */


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
        m_climber.rightWinch(0);
        m_climber.leftWinch(0);
    }
}
