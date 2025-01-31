package com.team5817.frc2025.autos;

import java.util.ArrayList;
import java.util.List;

import com.team5817.lib.motion.Trajectory;

public class TrajectoryLibrary {

    public static class l {
        public static List<Trajectory> trajectories = new ArrayList<>();
        public static Trajectory leftToHuman = new Trajectory("LeftToHuman");
        public static Trajectory humanToApp = new Trajectory("HumanTOApp");

    
        
        public static void update(){
            for(Trajectory t : trajectories){
                t.update();
            }
        }
        
    }

}
