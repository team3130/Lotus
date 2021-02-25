package frc.team3130.robot.sensors.vision;

import edu.wpi.first.wpilibj.DriverStation;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;

import java.util.ArrayList;

public class PixyCam {

    private Pixy2 m_pixy;
    private boolean isPixyConnected;
    private final int targetError = 10;
    private final int xC3 = 215; //x-value of point C3 (ball we look at for Path A red)
    private final int yC3 = 167;  //y-val of C3
    private final int xD5 = 0; //TODO: find real values/real ball
    private final int yD5 = 0;
    //pixy cam res is 315 x 207 (horizontal x vertical)

    public PixyCam(Link link){
        try{
        m_pixy = Pixy2.createInstance(link);
        m_pixy.init();
        isPixyConnected = true;
        }
        catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            isPixyConnected = false;
        }
    }

    public PixyCam(Link link, int arg){
        try{
            m_pixy = Pixy2.createInstance(link);
            m_pixy.init(arg);
            isPixyConnected = true;
        }
        catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            isPixyConnected = false;
        }
    }

    private boolean isBallHere(Block targetBlock, int xTarget, int yTarget){
       if(isPixyConnected) {
           if (targetBlock.getX() >= xTarget - targetError && targetBlock.getX() <= xTarget + targetError) {
               if (targetBlock.getY() >= yTarget - targetError && targetBlock.getY() <= yTarget + targetError) {
                   return true;

               }
           }
           return false;
       }
       System.out.println("Pixy isn't connected. Returning false(isBallHere()) This is a [REDACTED] moment");
       return false;
    }
    //path should be either "A" or "B" depending on the path
    Block largestBlock = largestBlock();
    public boolean isRedPath(String path){
       if(isPixyConnected){
        if (path.equals("A")){
            if(isBallHere(largestBlock, xC3,yC3)){
                return true;
            }
            else{return false;}
        }
        else if (path.equals("B")){
            if(isBallHere(largestBlock,xD5,yD5)){
                return true;
            }
            else{return false;}
        }
        else{
            System.out.println("Either: Neither A nor B was input for the method or my code broke. returning false (path blue) YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
            return false;
        }
       }
       else{
           System.out.println("Pixy isn't connected. Returning false(isRedPath()) (blue has been set to path). This is a [REDACTED] moment");
           return false;
       }

    }

    private Block largestBlock() {

        if (isPixyConnected){
            // System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found

            int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 12);
        if (blockCount <= 0) {
            return null; // If blocks were not found, stop processing
        }
        ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
        Block largestBlock = null;
        for (Block block : blocks) { // Loops through all blocks and finds the widest one
            if (largestBlock == null) {
                largestBlock = block;
            } else if (block.getWidth() > largestBlock.getWidth()) {
                largestBlock = block;
            }
        }
        return largestBlock;
    }
        else {
            System.out.println("Pixy isn't connected. Returning null (largestBlock()). This is a [REDACTED] moment");
            return null;
        }
    }

    public Pixy2 getPixy(){return m_pixy;}

}

