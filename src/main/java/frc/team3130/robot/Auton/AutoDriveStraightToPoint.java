package frc.team3130.robot.Auton;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


import java.util.Set;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Chassis;

public class AutoDriveStraightToPoint implements Command {

    private double m_distance;
    private double m_threshold;
    private double m_speed;
    private boolean m_shiftLow;

    private final Set<Subsystem> subsystems;

    private PIDController m_controller;

    /**
     * Creates a new AutoDriveStraightToPoint.
     */
    public AutoDriveStraightToPoint() {
        this.subsystems = Set.of(Chassis.getInstance());
        m_controller = new PIDController(1, 0, 0);
    }

    /**
     * Sets the command's parameters
     *
     * @param setpoint    distance to travel (inches)
     * @param threshold   how many inches within the setpoint
     * @param speed		  percentVBus to drive at
     * @param shiftLow	  whether the bot is in high gear or not
     */
    public void SetParam(double setpoint, double threshold, double speed, boolean shiftLow){
        //System.out.println("Param Set");
        m_distance = setpoint;
        m_threshold = threshold;
        m_speed = speed;
        m_shiftLow = shiftLow;
    }

    @Override
    public void initialize(){
        System.out.println("StartAutoDrive");
        m_controller.reset();

        Chassis.shift(false);
        Chassis.holdAngle(0,false);
        m_controller.setSetpoint(m_distance+Chassis.getDistance());
        m_controller.setTolerance(m_threshold);
        setPID();
        Chassis.configBrakeMode(true);
        Chassis.configRampRate(3);

    }

    private void useOutput(double output) {
        if(output>m_speed) output = m_speed;
        else if(output<-m_speed) output = -m_speed;

        Chassis.driveStraight(output);
    }

    private void setPID(){
        m_controller.setPID( //TODO:Tune PID, using Itasca values
                Preferences.getInstance().getDouble("DriveStraightP", 0.3),
                Preferences.getInstance().getDouble("DriveStraightI", 0),
                Preferences.getInstance().getDouble("DriveStraightD", 0)
        );
    }

    @Override
    public void execute() {
        System.out.println("Pos: "+Chassis.getDistance());
        System.out.println("Setpoint: "+m_controller.getSetpoint());
        System.out.println();
        //Chassis.driveStraight(0.5);
        useOutput(m_controller.calculate(Chassis.getDistance()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return false;
        return Math.abs(Chassis.getDistance() - m_controller.getSetpoint()) < m_threshold;
        //return m_controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING");
        Chassis.ReleaseAngle();
        Chassis.driveTank(0, 0, false);
        Chassis.configRampRate(RobotMap.kDriveMaxRampRate);
    }

    /**
     * <p>
     * Specifies the set of subsystems used by this command.  Two commands cannot use the same
     * subsystem at the same time.  If the command is scheduled as interruptible and another
     * command is scheduled that shares a requirement, the command will be interrupted.  Else,
     * the command will not be scheduled. If no subsystems are required, return an empty set.
     * </p><p>
     * Note: it is recommended that user implementations contain the requirements as a field,
     * and return that field here, rather than allocating a new set every time this is called.
     * </p>
     *
     * @return the set of subsystems that are required
     */
    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}