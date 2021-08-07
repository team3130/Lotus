package frc.team3130.robot.SupportingClasses;

import java.util.*;

public class BalManager implements Comparable<Bal>{
    private final PriorityQueue<Bal> balPriorityQueue;

    /**
     * <h1>Constructs the bal manager with 0 parameters</h1>
     * Constructs the bal priority queue and passes in the overridden compareTo method as a method reference
     */
    public BalManager() {
        balPriorityQueue = new PriorityQueue<Bal>(Comparator.comparing(this::compareTo));
    }

    /**
     * Adds ball if not exists
     * @param bal ball to be added
     * @return if successful or not, will be false if ball was already in the queue
     */
    public boolean addBall(Bal bal) {
        // adds ball if not in thr array
        if (!balPriorityQueue.contains(bal)) {
            balPriorityQueue.add(bal);
            return true;
        }
        // if in the array it updates the fixed position with an avg as calculated by the nano
        updateBall(bal);
        return false;
    }

    /**
     *  gets closest ball
     * @return returns null if the list is empty and if not it returns the closest ball without removing it
     */
    public Bal getClosestBall() {
        try {
            return balPriorityQueue.peek();
        }
        catch (Exception e) {
            System.out.println("No balls can be found ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
            return null;
        }
    }

    /**
     * Used to update the values of balls
     * @param bal bal
     */
    public void updateBall(Bal bal) {
        for (Bal baller : balPriorityQueue) {
            // will run if it sees ball and only if they have different fixed positions
            if (bal.equals(baller) && !Arrays.equals(bal.getPositionFix(), baller.getPositionFix())) {
                // updates the position of the ball
                baller.updatePositionFix(bal.getPositionFix());
                // break out to save system resources
                break;
            }
        }
    }

    /**
     * iterates through parameter bals to update each one
     * @param bals collection of balls being compared
     */
    public void updateBalls(Collection<Bal> bals) {
        // for each loop to update every ball provided in the collection
        for (Bal bal : bals) {
            updateBall(bal);
        }
    }

    /**
     *  removes and returns a ball
     * @return the ball removed
     */
    public Bal pop() {
        return balPriorityQueue.remove();
    }

    /**
     * Removes a ball from the queue
     * @param bal the bal that is being removed
     */
    public void removeBal(Bal bal) {
        balPriorityQueue.remove(bal);
    }

    /**
     * Overridden "compareTo" to compare objects that the queue will use.
     * The plan for this method is to incorporate more logic to decide whether the ball is in close proximity to other balls\
     * and return a value that incorporates both the distance to the ball, and the distance from the ball to other balls\
     * as well as how many balls we have/need to collect
     * @param o Object that will be compared to
     * @return an integer value that will be used by the priority heap
     */
    @Override
    public int compareTo(Bal o) {
        //TODO: Make this return a value that incorporates the balls proximity to other balls
        return (int) o.getDistance();
    }
}
