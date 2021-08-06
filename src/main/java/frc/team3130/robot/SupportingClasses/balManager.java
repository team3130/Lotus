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
    private HashMap<Integer, Integer> idToIndex = new HashMap<Integer, Integer>();
    private ArrayList<Bal> Balls = new ArrayList<>();

    public void updateBalls() {}

    public void addBall(Bal bal) {
        boolean isIN = false;
        for (int looper = 0; looper < Balls.size(); looper++) {
            if (bal.getPositionFix() == Balls.get(looper).getPositionFix() || Balls.get(looper).getPositionFix()[0] + 0.5 == bal.getPositionFix()[0] || Balls.get(looper).getPositionFix()[0] - 0.5 == bal.getPositionFix()[0] || Balls.get(looper).getPositionFix()[1] + 0.5 == bal.getPositionFix()[1] || Balls.get(looper).getPositionFix()[1] - 0.5 == bal.getPositionFix()[1]) {
                isIN = true;
            }
            else if (bal.getId() == Balls.get(looper).getId()) {
                isIN = true;
            }
        }
        if (!isIN) {
            int index = Balls.size();
            Balls.add(bal);
            
        }
    }

    public Bal getClosest() {

    }

    private double getDistance(double[] arr) {
        return Math.sqrt((arr[0] * arr[0]) + (arr[1] * arr[1]));
    }

}
