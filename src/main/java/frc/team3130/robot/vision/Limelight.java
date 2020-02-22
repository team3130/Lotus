package frc.team3130.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Turret;


public class Limelight {
    //Instance Handling
    private static Limelight m_pInstance;

    public static Limelight GetInstance() {
        if (m_pInstance == null) m_pInstance = new Limelight();
        return m_pInstance;
    }

    NetworkTable visionTable;

    private NetworkTableEntry tv; // Whether the limelight has any valid targets (0 or 1)
    private NetworkTableEntry tx; // x angle offset from crosshair, range of -27 to 27
    private NetworkTableEntry ty; // y angle offset from crosshair, range of -20.5 to 20.5
    private NetworkTableEntry ta; // area of contour bounding box
    private NetworkTableEntry ts; // Skew or rotation (-90 degrees to 0 degrees)


    private MedianFilter txFilter;
    private MedianFilter tyFilter;
    private double x_targetOffsetAngle;
    private double y_targetOffsetAngle;
    private double area;
    private double skew;

    private Matrix<N3, N3> rotation;      // Own rotation
    private Matrix<N3, N1> translation;   // Own translation
    private Matrix<N3, N1> tilt;          // Turret tilt
    private Matrix<N3, N1> realVector;

    protected Limelight() {
        visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = visionTable.getEntry("tv");
        tx = visionTable.getEntry("tx");
        ty = visionTable.getEntry("ty");
        ta = visionTable.getEntry("ta");
        ts = visionTable.getEntry("ts");

        txFilter = new MedianFilter(RobotMap.kLimelightFilterBufferSize);
        tyFilter = new MedianFilter(RobotMap.kLimelightFilterBufferSize);
        x_targetOffsetAngle = 0.0;
        y_targetOffsetAngle = 0.0;
        area = 0.0;
        skew = 0.0;

        Matrix<N3, N1> rVec = Algebra.buildVector(
                Math.toRadians(RobotMap.kLimelightPitch),
                Math.toRadians(RobotMap.kLimelightYaw),
                Math.toRadians(RobotMap.kLimelightRoll)
        );
        rotation = Algebra.Rodrigues(rVec);
        translation = Algebra.buildVector(
                RobotMap.kLimelightOffset,
                RobotMap.kLimelightHeight,
                RobotMap.kLimelightLength
        );
        tilt = Algebra.buildVector(
            Math.toRadians(RobotMap.kTurretPitch),
            0,
            Math.toRadians(RobotMap.kTurretRoll)
        );
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    public double getTy() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public double getSkew() {
        return ts.getDouble(0.0);
    }

    /**
     * Limelight vision tracking pipeline latency.
     * @return Latency in milliseconds
     */
    public double getLatency(){
        return visionTable.getEntry("tl").getDouble(0) + RobotMap.kLimelightLatencyMs;
    }

    /**
     * Read data from the Limelight and update local values
     */
    public void updateData() {
        x_targetOffsetAngle = txFilter.calculate(getTx());
        y_targetOffsetAngle = tyFilter.calculate(getTy());
        area = getArea();
        skew = getSkew();
        realVector = calcPosition(x_targetOffsetAngle, y_targetOffsetAngle);
    }

    /**
     * Calculate additional rotation matrix due to robot's tilt
     * If the plane in which the turret is turning is not level
     * then we need to compensate for this tilt.
     * @param heading the angle in degrees from the turret's encoder
     * @return rotation matrix to compensate for the tilt
     */
    Matrix<N3,N3> turretRotation(double heading) {
        // Heading is where the turret is facing relative to the robot
        // so convert it to the robot's heading relative to the turret (negative)
        // and create a rotation matrix from this rotation
        Matrix<N3,N3> turn = Algebra.Rodrigues(
            Algebra.buildVector(0, Math.toRadians(-heading), 0)
        );
        // Now using this new rotation rotate the tilt vector and create
        // another rotation matrix which is the compensation rotation we want
        return Algebra.Rodrigues(turn.times(tilt));
    }

    /**
     * Build a "unit" vector in 3-D and rotate it from camera's
     * coordinates to real (robot's (turret's)) coordinates
     *
     * @param ax horizontal angle, left is positive
     * @param ay vertical angle, up is positive
     * @param heading turret's turn angle for tilt compensation
     * @return a vector pointing to the direction but with the length = 1
     */
    public Matrix<N3, N1> levelVector(double ax, double ay, double heading) {
        // Convert degrees from the vision to unit coordinates
        double ux = Math.tan(Math.toRadians(ax));
        double uy = Math.tan(Math.toRadians(ay));
        // Do two rotations: for LimeLight mount and for robot tilt
        return turretRotation(heading).times(
            rotation.times(
                Algebra.buildVector(ux, uy, 1)
            )
        );
    }

    /**
     * Calculate a position vector based on angles from vision
     *
     * @param ax Horizontal Offset From Crosshair To Target
     * @param ay Vertical Offset From Crosshair To Target
     * @return resulting vector from the Turret's origin to the target
     */
    public Matrix<N3, N1> calcPosition(double ax, double ay) {

        // Find where the vector is actually pointing
        Matrix<N3, N1> v0 = levelVector(ax, ay, Turret.getAngleDegrees());

        // Scaling ratio based on the known height of the vision target
        double c = (RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight) / v0.get(1, 0);

        // Find the real vector from camera to target
        Matrix<N3, N1> v = v0.times(c);

        // Add the offset of the camera from the turret's origin
        Matrix<N3, N1> a = translation.plus(v);
        // That's the droid we're looking for
        return a;
    }

    /**
     * If the Limelight has a target track
     *
     * @return true if Limelight has targets
     */
    public boolean hasTrack() {
        return tv.getDouble(0.0) == 1.0;
    }

    public double getTargetRotationTan() {
        double realSkew = Math.toRadians(skew < -45 ? skew + 90 : skew);
        // Very approximate adjustment for the camera tilt, should work for small angles
        // Rationale: the best view is straight from below which is 90 degree, then no adjustment would be needed
        // Then it gets worse as the tilt comes closer to zero degree - can't see rotation at the horizon.
        // Ideally it would be better to do this with vectors and matrices
        // TAN(new) = COS(ty)*TAN(skew)/SIN(cam+ty)
        double tx = Math.toRadians(x_targetOffsetAngle);
        double ty = Math.toRadians(y_targetOffsetAngle);
        double cam = Math.toRadians(RobotMap.kLimelightPitch);
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
    public double getDegHorizontalError() {
        // realVector is calculated in updateData that should be called before doing this
        // The horizontal error is an angle between the vector's projection on the XZ plane
        // and the Z-axis which is where the turret is always facing.
        return Math.toDegrees(Math.atan2(realVector.get(0, 0), realVector.get(2, 0)));
    }

    /**
     * Get the ground distance to the target
     *
     * @return distance in inches
     */
    public double getDistanceToTarget() {
        if (Limelight.GetInstance().hasTrack()) {
            Matrix<N3, N1> projection = realVector.copy();
            projection.set(1, 0, 0.0);
            return projection.normF();
        } else {
            return 0.0;
        }
    }


    /**
     * Calibrate the tilt angle of the Limelight
     */
    public double calibrate() {
        updateData();

        double height = RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight;
        double distance = RobotMap.kLimelightCalibrationDist;

        double tiltAngle = Math.toDegrees(Math.atan2(height, distance)) - y_targetOffsetAngle;
        return tiltAngle;
    }

    /**
     * Turn the LEDs of the Limelight on or off
     *
     * @param isOn true means on
     */
    public void setLedState(boolean isOn) {
        if (isOn) {
            visionTable.getEntry("ledMode").setNumber(3);
        } else {
            visionTable.getEntry("ledMode").setNumber(1);
        }
    }

    /**
     * Set the current vision tracking pipeline of the Limelight
     *
     * @param pipelineNumber the id of the pipeline
     */
    public void setPipeline(double pipelineNumber) {
        visionTable.getEntry("pipeline").setNumber(pipelineNumber);
        /*
        How to set a parameter value:
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<PUT VARIABLE NAME HERE>").setNumber(<TO SET VALUE>);
        */
    }


    public static void outputToShuffleboard() {
        Limelight o = GetInstance();
        SmartDashboard.putNumber("Limelight Filtered X Angle", o.x_targetOffsetAngle);
        SmartDashboard.putNumber("Limelight Filtered Y Angle", o.y_targetOffsetAngle);
        SmartDashboard.putNumber("Limelight Distance", o.getDistanceToTarget());
        SmartDashboard.putNumber("LimelightArea", o.area);
        SmartDashboard.putBoolean("Limelight Has Target", o.hasTrack());
        SmartDashboard.putNumber("Limelight mounting angle", o.calibrate());
    }

}
