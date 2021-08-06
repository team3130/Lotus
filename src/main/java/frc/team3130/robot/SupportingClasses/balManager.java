package frc.team3130.robot.SupportingClasses;

import java.util.*;

/**
 * <p>
 *     Class to:
 *          update Relative location of balls
 *          Store the sorted balls in a DS
 * </p>
 */
public class balManager {
    private ArrayList<Bal> Balls = new ArrayList<>();

    private int closestIndex = -1;

    public void updateBalls() {}

    /**
     * <p>checks if the ball object is already in the array list and accounts for a 0.5 precision error</p>
     * @param bal Ball objects to be added
     */
    public void addBall(Bal bal) {
        boolean isIN = false;
        for (int looper = 0; looper < Balls.size(); looper++) {
            if (bal.getPositionFix() == Balls.get(looper).getPositionFix() || Balls.get(looper).getPositionFix()[0] + 0.5 <= bal.getPositionFix()[0] && Balls.get(looper).getPositionFix()[0] - 0.5 >= bal.getPositionFix()[0] || Balls.get(looper).getPositionFix()[1] + 0.5 >= bal.getPositionFix()[1] || Balls.get(looper).getPositionFix()[1] - 0.5 <= bal.getPositionFix()[1]) {
                isIN = true;
            }
            else if (bal == Balls.get(looper)) {
                isIN = true;
            }
        }
        if (!isIN) {
            int index = Balls.size();
            Balls.add(bal);

            if (closestIndex == -1) {
                closestIndex = index;
            }
            else {
                try {
                    if (Balls.get(closestIndex).getDistance() == bal.getDistance()) {
                        closestIndex = index;
                    }
                }
                catch (Exception exc) {
                    // closestID not found
                    closestIndex = -1;
                }
            }
        }
    }

    //TODO: write logic to get closest clump of balls to maximize number you can grab
    public Bal getClosest() {
        return Balls.get(closestIndex);
    }

    private double getDistance(double[] arr) {
        return Math.sqrt((arr[0] * arr[0]) + (arr[1] * arr[1]));
    }

}
