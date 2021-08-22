package frc.team3130.robot.SupportingClasses;

import frc.team3130.robot.subsystems.Chassis;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.*;

public class BalManager implements Comparator<Bal>{
    private final PriorityQueue<Bal> balPriorityQueue;
    private final Chassis m_chassis;

    private final double[][] rotation = {
            {-0.06815984, -0.87361327},
            {-0.38507791,  0.14232387}
    };

    private final double[] translation = {216.85726298,  63.07897139};

    /**
     * <h1>Constructs the bal manager with 0 parameters</h1>
     * Constructs the bal priority queue and passes in the overridden compareTo method as a method reference
     */
    public BalManager(Chassis chassis) {
        balPriorityQueue = new PriorityQueue<>(this);
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
     * Time Complexity: O(n^2) <- includes call to updateBall()
     */
    public void updateBalls(Collection<Bal> bals) {
        // for each loop to update every ball provided in the collection
        for (Bal bal : bals) {
            updateBall(bal);
        }
    }

    public double[] predict(double[] pixel) {
        double[] coords = new double[2];
        coords[0] = ((pixel[0] * rotation[0][0]) + (pixel[1] * rotation[0][1])) + translation[0];
        coords[1] = ((pixel[0] * rotation[1][0]) + (pixel[1] * rotation[1][1])) + translation[1];
        return coords;
    }

    public void addBalls(Collection<Pixy2CCC.Block> bals) {
        // for each loop to add every ball provided in the collection
        for (Pixy2CCC.Block bal : bals) {
            double[] coords = {bal.getX(), bal.getY()};
            predict(coords);
            balPriorityQueue.add(new Bal(coords));
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
                if (bal.getPositionRel().get()[0] < furthestLeft.getPositionRel().get()[0]) {furthestLeft = bal;}
                else if(bal.getPositionRel().get()[0] > furthestRight.getPositionRel().get()[0]) {furthestRight = bal;}
            }
        }
        for (Bal toBeCompared : balPriorityQueue) {
            // if the ball that is to be compared is in the range of the furthest on the left and the furthest on the right adn is not there, remove it from the queue
            if (toBeCompared.getPositionRel().get()[0] >= furthestLeft.getPositionRel().get()[0] && toBeCompared.getPositionRel().get()[0] <= furthestRight.getPositionRel().get()[0]) {
                balPriorityQueue.remove(toBeCompared);
            }
        }
    }

    /**
     * Overridden "compareTo" to compare objects that the queue will use.
     * The plan for this method is to incorporate more logic to decide whether the ball is in close proximity to other balls\
     * and return a value that incorporates both the distance to the ball, and the distance from the ball to other balls\
     * as well as how many balls we have/need to collect
     * @param t1 Object that will be compared to bal
     * @param bal Object that will be compared to t1
     * @return an integer value that will be used by the priority heap
     */
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
