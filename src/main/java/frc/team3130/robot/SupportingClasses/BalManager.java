package frc.team3130.robot.SupportingClasses;

import java.util.Comparator;
import java.util.PriorityQueue;

public class BalManager implements Comparable<Bal>{
    private final PriorityQueue<Bal> balPriorityQueue;

    public BalManager() {
        balPriorityQueue = new PriorityQueue<Bal>(Comparator.comparing(this::compareTo));
    }

    public boolean addBall(Bal bal) {
        if (!balPriorityQueue.contains(bal)) {
            balPriorityQueue.add(bal);
            return true;
        }
        return false;
    }

    public Bal getClosestBall() {
        try {
            return balPriorityQueue.peek();
        }
        catch (Exception e) {
            System.out.println("No balls can be found ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
            return null;
        }
    }

    public Bal pop() {
        return balPriorityQueue.remove();
    }

    public void removeBal(Bal bal) {
        balPriorityQueue.remove(bal);
    }

    @Override
    public int compareTo(Bal o) {
        return (int) o.getDistance();
    }

}
