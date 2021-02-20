package frc.team3130.robot.sensors.vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;

public class PixyCam {

    private final Pixy2 pixy;

    public PixyCam(Link link){
        pixy = Pixy2.createInstance(link);
        pixy.init();
    }

    public PixyCam(Link link, int arg){
        pixy = Pixy2.createInstance(link);
        pixy.init(arg);
    }



    public Pixy2 getPixy(){return pixy;}

}

