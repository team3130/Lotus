package frc.team3130.robot.vision;

import org.junit.Test;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import frc.team3130.robot.RobotMap;

import static org.junit.Assert.assertEquals;

public class testLimeLight {
    @Test
    public void TiltTest() {
        Matrix<N3,N3> turMatrix = Limelight.GetInstance().turretRotation(90);
        for (int i = 0; i < 3; i++) {
            System.out.println(String.format(" [%8.5f, %8.5f, %8.5f]",
            turMatrix.get(i,0), turMatrix.get(i,1), turMatrix.get(i,2)));
        }
    }

    @Test
    public void RotationTest() {
        double ay = -22.6;
        double ax = -3.1;
        Matrix<N3,N1> lvec = Limelight.GetInstance().levelVector(ax, ay, 0);
        double norm = Math.sqrt(1 + Math.pow(Math.tan(Math.toRadians(ax)), 2) + Math.pow(Math.tan(Math.toRadians(ay)), 2));
        System.out.println(String.format("level Vec = [%8.5f, %8.5f, %8.5f], norm = %8.5f",
            lvec.get(0,0), lvec.get(1,0), lvec.get(2,0), norm));
        assertEquals("Length is wrong", norm, lvec.normF(), 0.001);
    }

    @Test
    public void PositionTest() {
        double ay = 10;
        double distance = (RobotMap.VISIONTARGETHEIGHT - RobotMap.kLimelightHeight)
            / Math.tan(Math.toRadians(ay - RobotMap.kLimelightPitch - RobotMap.kTurretPitch))
            + RobotMap.kLimelightLength;
        System.out.println(String.format("distance = %8.5f", distance));
        Matrix<N3,N1> realVec = Limelight.GetInstance().calcPosition(0, ay);
        System.out.println(String.format("realVec = [%8.5f, %8.5f, %8.5f]",
            realVec.get(0,0),realVec.get(1,0),realVec.get(2,0)));
        assertEquals("Y is wrong",    98.25, realVec.get(1,0), 0.01);
        assertEquals("Z is wrong", distance, realVec.get(2,0), 0.2);
    }
}