package frc.team3130.robot.vision;

import org.junit.Test;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

import static org.junit.Assert.assertEquals;

public class testAlgebra {
    @Test
    public void RodriguesTest() {
        // Let's try to rotate something one radian around X axis in positive direction
        Matrix<N3,N1> rVec = new VecBuilder<>(Nat.N3()).fill(1,0,0);
        // Let's rotate the Y-unit vector, where Y=1 and other coordinates are 0s.
        Matrix<N3,N1> aVec = new VecBuilder<>(Nat.N3()).fill(0,1,0);
        // Make a rotation matrix using the Rodrigues formula
        Matrix<N3,N3> rotation = Algebra.Rodrigues(rVec);
        // And rotate the given vector by multiplying it by the rotation matrix
        Matrix<N3,N1> cVec = rotation.times(aVec);
        // Let's see the resulting vector on the console
        System.out.println(String.format("cVec = [%8.5f, %8.5f, %8.5f]",
            cVec.get(0,0),cVec.get(1,0),cVec.get(2,0)));
        // In the result the Y-unit vector should lean toward the positive Z-axis exactly one radian.
        // Check the results for the unit testing (see JUnit docs)
        assertEquals("X is wrong", 0.0,         cVec.get(0,0), 0.0001);
        assertEquals("Y is wrong", Math.cos(1), cVec.get(1,0), 0.0001);
        assertEquals("Z is wrong", Math.sin(1), cVec.get(2,0), 0.0001);
    }
}