package frc.team3130.robot.util;

public class Util {

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) { return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param val    value to trim
     * @param deadband range around zero
     */
    public static double applyDeadband(double val, double deadband){
        if(Math.abs(val) > deadband){
            if(val > 0.0){
                return (val - deadband)/ (1.0 - deadband);
            } else {
                return (val + deadband)/ (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

}
