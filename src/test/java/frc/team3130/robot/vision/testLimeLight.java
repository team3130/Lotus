package frc.team3130.robot.vision;

import org.junit.Test;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

import static org.junit.Assert.assertEquals;

public class testLimeLight {
    @Test
    public void PositionTest() {
        Matrix<N3,N1> realVec = Limelight.GetInstance().calcPosition(0, 0);
        System.out.println(String.format("realVec = [%8.5f, %8.5f, %8.5f]",
            realVec.get(0,0),realVec.get(1,0),realVec.get(2,0)));
        assertEquals("X is wrong", 0.0,         realVec.get(0,0), 0.0001);
    }
}