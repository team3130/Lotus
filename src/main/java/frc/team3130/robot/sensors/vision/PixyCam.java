package frc.team3130.robot.sensors.vision;

import edu.wpi.first.wpilibj.DriverStation;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;

import java.util.ArrayList;

public class PixyCam {

    private Pixy2 m_pixy;
    private boolean m_isPixyConnected;
    private final int m_targetError = 10; //10 pixels of lenience for checking where ball is
    private final int xB3 = 215; //x-value of point B3 (ball we look at for Path B red)
    private final int yB3 = 167;  //y-val of B3
    private final int xD5 = 0; //TODO: find real values
    private final int yD5 = 0;
    //pixy cam res is 315 x 207 (horizontal x vertical)

    public PixyCam(Link link){
        try{
        m_pixy = Pixy2.createInstance(link);
        m_pixy.init();
        m_isPixyConnected = true;
        }
        catch (Exception ex) {
            //If connection fails log the error
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            m_isPixyConnected = false;
        }
    }

    public PixyCam(Link link, int arg){
        try{
            m_pixy = Pixy2.createInstance(link);
            m_pixy.init(arg);
            m_isPixyConnected = true;
        }
        catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Pixy didn't get constructed right. This is a [REDACTED] moment " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            m_isPixyConnected = false;
        }
    }

    private boolean isBallHere(Block targetBlock, int xTarget, int yTarget){
       if(m_isPixyConnected) {
           if (targetBlock.getX() >= xTarget - m_targetError && targetBlock.getX() <= xTarget + m_targetError) {
               if (targetBlock.getY() >= yTarget - m_targetError && targetBlock.getY() <= yTarget + m_targetError) {
                   return true;

               }
           }
           return false;
       }
       String str_err = "Pixy isn't connected. Returning false(isBallHere()) This is a [REDACTED] moment";
        DriverStation.reportError(str_err, false);
       return false;
    }
    //path should be either "A" or "B" depending on the path
    //Block largestBlock = largestBlock();

    public boolean isRedPath(String path){
        if(m_isPixyConnected) {
           try{
               int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 3);
               ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache();

            if(blockCount <= 0){
                String str_error = "Pixy didn't detect any blocks returning false (blue path)";
                DriverStation.reportError(str_error, false);
            }

               for (Block block: blocks) {

                    if (path.equals("A")) {
                        if (isBallHere(block, xD5, yD5)) {
                            return true;
                        }
                    } else if (path.equals("B")) {
                        if (isBallHere(block, xB3, yB3)) {
                            return true;
                        }
                    } else {
                        System.out.println("Either: Neither A nor B was input for the method or my code broke. returning false (path blue) YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
                        return false;
                    }


                }
                    } catch (Exception ex) {
                    String str_error = "Pixy or one of its objects was null. Returning false (blue path). " + ex.getLocalizedMessage();
                    DriverStation.reportError(str_error, false);
                    return false;
                }
            }


        else{
           String str_err = "Pixy isn't connected. Returning false(isRedPath()) (blue has been set to path). This is a [REDACTED] moment";
           DriverStation.reportError(str_err, false);
            return false;
        }

        return false; //returns false if no block was found in the indicated position

    }

    private Block largestBlock() {

        if (m_isPixyConnected){
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
            String str_err = "Pixy isn't connected. Returning null (largestBlock()). This is a [REDACTED] moment";
            DriverStation.reportError(str_err, false);

            return null;
        }
    }



    public Pixy2 getPixy(){return m_pixy;}

}

