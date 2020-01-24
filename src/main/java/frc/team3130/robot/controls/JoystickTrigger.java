package frc.team3130.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTrigger extends Trigger {

    private Joystick stick;
    private int axis;
    private double threshold;

    public JoystickTrigger(Joystick stick, int axis) {
        this.stick = stick;
        this.axis = axis;
        threshold = 0.1;
    }

    public JoystickTrigger(Joystick stick, int axis, double threshold) {
        this.stick = stick;
        this.axis = axis;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return stick.getRawAxis(axis) > threshold;
    }
}