package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;

import java.util.*;

public class BalManager implements Comparator<Bal>{
    private final PriorityQueue<Bal> balPriorityQueue;
    private final Chassis m_chassis;

    /**
     * <h1>Constructs the bal manager with 0 parameters</h1>
     * Constructs the bal priority queue and passes in the overridden compareTo method as a method reference
     */
    public BalManager(Chassis chassis) {
        balPriorityQueue = new PriorityQueue<Bal>(this);
        m_chassis = chassis;
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
     * @warning Make sure to perform a sanity check and make sure the ball at the position can be tracked before running the ramsete command
     * @return returns null if the list is empty and if not it returns the closest ball without removing it
     * Time Complexity: Constant
     */
    public Bal getClosestBall() {
        try {
            return balPriorityQueue.peek();
        }
        catch (Exception e) {
            System.out.println("No balls can be found ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR, Queue is empty");
            return null;
        }
    }

    /**
     * Used to update the values of balls
     * @param bal bal
     * Time Complexity: O(n)
     */
    public void updateBall(Bal bal) {
        for (Bal baller : balPriorityQueue) {
            // will run if it sees ball and only if they have different fixed positions
            if (bal.equals(baller) && !Arrays.equals(bal.getPositionRel(), baller.getPositionRel())) {
                // updates the position of the ball
                baller.updatePositionRel(bal.getPositionRel());
                // break out to save system resources
                break;
            }
        }
    }

    /**
     * iterates through parameter bals to update each one
     * @param bals collection of balls being compared
     * Time Complexity: O(n^2) <- includes call to updateBall()
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
     * checks if each ball in the queue is still there (if it would be able to see it)
     * should be called every 10-20 iterations of periodic (use a char var and increment it every time until it is a certain value. to be done on call)
     * sorts the balls in the parameter based on their relative x position
     * @param balls The current balls in the frame
     */
    public void checkIfStillThere(Collection<Bal> balls) {
        // if there are too little balls to check
        if (balls.isEmpty() || balls.size() < 2) {
            return;
        }
        Bal furthestLeft = null,
                furthestRight = null;
        for (Bal bal : balls) {
            // if is there return without doing anything
            if (balPriorityQueue.contains(bal)) {
                return;
            }

            // checks if first iteration
            if (furthestLeft == null) {
                furthestLeft = bal;
                furthestRight = bal;
            }
            else {
                if (bal.getPositionAbs().get()[0] < furthestLeft.getPositionAbs().get()[0]) {furthestLeft = bal;}
                else if(bal.getPositionAbs().get()[0] > furthestRight.getPositionAbs().get()[0]) {furthestRight = bal;}
            }
        }
        for (Bal toBeCompared : balPriorityQueue) {
            // if the ball that is to be compared is in the range of the furthest on the left and the furthest on the right adn is not there, remove it from the queue
            if (toBeCompared.getPositionAbs().get()[0] >= furthestLeft.getPositionAbs().get()[0] && toBeCompared.getPositionAbs().get()[0] <= furthestRight.getPositionAbs().get()[0]) {
                balPriorityQueue.remove(toBeCompared);
            }
        }
    }

    @Override
    public int compare(Bal bal, Bal t1) {
        if (bal.getDistance() > t1.getDistance()) {
            return 1;
        }
        if (bal.equals(t1) || bal.getDistance() == t1.getDistance()) {
            return 0;
        }
        else return -1;
    }
}
