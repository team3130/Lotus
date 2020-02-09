package frc.team3130.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import frc.team3130.robot.RobotMap;


public class Limelight {
    //Instance Handling
    private static Limelight m_pInstance;

    public static Limelight GetInstance() {
        if (m_pInstance == null) m_pInstance = new Limelight();
        return m_pInstance;
    }

    private static NetworkTableEntry tv; // Whether the limelight has any valid targets (0 or 1)
    private static NetworkTableEntry tx; // x angle offset from crosshair, range of -27 to 27
    private static NetworkTableEntry ty; // y angle offset from crosshair, range of -20.5 to 20.5
    private static NetworkTableEntry ta; // area of contour bounding box
    private static NetworkTableEntry ts; // Skew or rotation (-90 degrees to 0 degrees)

    private static double x_targetOffsetAngle = 0.0;
    private static double y_targetOffsetAngle = 0.0;
    private static double area = 0.0;
    private static double skew = 0.0;

    private static Matrix<N3,N3> rotation;      // Own rotation
    private static Matrix<N3,N1> translation;   // Own translation

    protected Limelight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");

        Matrix<N3,N1> rVec = new VecBuilder<>(Nat.N3()).fill(
            Math.toRadians(RobotMap.kLimeLightPitch),
            Math.toRadians(RobotMap.kLimeLightYaw),
            Math.toRadians(RobotMap.kLimeLightRoll)
        );
        rotation = Algebra.Rodrigues(rVec);
        translation = new VecBuilder<>(Nat.N3()).fill(
            RobotMap.kLimeLightOffset,
            RobotMap.kLimelightHeight,
            -RobotMap.kLimeLightLength
        );
    }

    /**
     * Read data from the Limelight and update local values
     */
    public static void updateData() {
        //Check if limelight sees a target
        x_targetOffsetAngle = -tx.getDouble(0.0);
        y_targetOffsetAngle = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        skew = ts.getDouble(0.0);
    }

    /**
     * If the Limelight has a target track
     *
     * @return true if Limelight has targets
     */
    public static boolean hasTrack() {
        return tv.getDouble(0.0) == 1.0;
    }

    public static double getTargetRotationTan() {
        double realSkew = Math.toRadians(skew < -45 ? skew + 90 : skew);
        // Very approximate adjustment for the camera tilt, should work for small angles
        // Rationale: the best view is straight from below which is 90 degree, then no adjustment would be needed
        // Then it gets worse as the tilt comes closer to zero degree - can't see rotation at the horizon.
        // Ideally it would be better to do this with vectors and matrices
        // TAN(new) = COS(ty)*TAN(skew)/SIN(cam+ty)
        double tx = Math.toRadians(x_targetOffsetAngle);
        double ty = Math.toRadians(y_targetOffsetAngle);
        double cam = Math.toRadians(RobotMap.kLimeLightPitch);
        double sinTilt = Math.sin(cam + ty);
        double tanRot = Math.cos(ty) * Math.tan(realSkew) / sinTilt + 10.0 * (1.0 - Math.cos(tx)) * sinTilt;
        System.out.format("Real skew:%8.3f, rot:%8.3f ty:%8.3f %n", realSkew, tanRot, ty);
        return tanRot;
    }

    /**
     * Get the horizontal angle error to target
     *
     * @return angle in degrees
     */
    public static double getDegHorizontalError() {
        return x_targetOffsetAngle;
    }

    /**
     * Get the ground distance to the target
     *
     * @return distance in inches
     */
    public static double getDistanceToTarget() {
        if (area == 0.0) return 0.0; // we have no target to track, return 0.0

        double angle = y_targetOffsetAngle + RobotMap.kLimeLightPitch;
        double hLimelight = RobotMap.kLimelightHeight;
        double hTarget = RobotMap.VISIONTARGETHEIGHT;

        return (hTarget - hLimelight) / Math.tan(Math.toRadians(angle));
    }


    /**
     * Calibrate the tilt angle of the Limelight
     */
    public static void calibrate() {
        updateData();

        double height = RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight;
        double distance = RobotMap.kLimeLightCalibrationDist;

        double tiltAngle = Math.toDegrees(Math.atan2(height, distance)) - y_targetOffsetAngle;
        System.out.format("Limelight Tilt angle: %f %n", tiltAngle);
    }

    /**
     * Turn the LEDs of the Limelight on or off
     *
     * @param isOn true means on
     */
    public static void setLedState(boolean isOn) {
        if (isOn) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        }
    }

    /**
     * Set the current vision tracking pipeline of the Limelight
     *
     * @param pipelineNumber the id of the pipeline
     */
    public static void setPipeline(double pipelineNumber) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
        /*
        How to set a parameter value:
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<PUT VARIABLE NAME HERE>").setNumber(<TO SET VALUE>);
        */
    }


    public static void outputToSmartDashboard() {
        SmartDashboard.putNumber("Limelight X Angle", x_targetOffsetAngle);
        SmartDashboard.putNumber("Limelight Y Angle", y_targetOffsetAngle);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("Limelight Has Target", hasTrack());
    }

}
