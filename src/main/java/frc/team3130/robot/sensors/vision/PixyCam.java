package frc.team3130.robot.sensors.vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;

import java.util.ArrayList;

public class PixyCam {

    private final Pixy2 pixy;
    private final int targetError = 10;
    private final int xC3 = 215; //x-value of point C3 (ball we look at for Path A red)
    private final int yC3 = 167;  //y-val of C3
    private final int xD5 = 0; //TODO: find real values/real ball
    private final int yD5 = 0;


    public PixyCam(Link link){
        pixy = Pixy2.createInstance(link);
        pixy.init();
    }

    public PixyCam(Link link, int arg){
        pixy = Pixy2.createInstance(link);
        pixy.init(arg);
    }

    private boolean isBallHere(Block targetBlock, int xTarget, int yTarget){
        if(targetBlock.getX() >= xTarget - targetError && targetBlock.getX() <= xTarget + targetError){
            if(targetBlock.getY() >= yTarget - targetError && targetBlock.getY() <= yTarget + targetError){
                return true;

            }
        }
        return false;
    }
    //path should be either "A" or "B" depending on the path
    Block largestBlock = largestBlock();
    public boolean isRedPath(String path){
        if (path.equals("A")){
            if(isBallHere(largestBlock, xC3,yC3)){
                return true;
            }
            else{return false;}
        }
        else if (path.equals("B")){
            if(isBallHere(largestBlock,xD5,xD5)){
                return true;
            }
            else{return false;}
        }
        else{
            System.out.println("Either: Neither A nor B was input for the method or my code broke. returning false (path blue) YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
            return false;
        }

    }

    private Block largestBlock(){

       // System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found

        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 12);
        if (blockCount <= 0) {
            return null; // If blocks were not found, stop processing
        }
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
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

    public Pixy2 getPixy(){return pixy;}

}

