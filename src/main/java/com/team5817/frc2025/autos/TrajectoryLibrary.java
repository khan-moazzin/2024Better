package com.team5817.frc2025.autos;

import java.util.ArrayList;
import java.util.List;

import com.team5817.lib.motion.Trajectory;

public class TrajectoryLibrary {

    public static class l {
        public static List<Trajectory> trajectories = new ArrayList<>();
        public static Trajectory TSTo3A = new Trajectory("STo3A");
        public static Trajectory T3AToH = new Trajectory("3AToH");
        public static Trajectory THTO7A = new Trajectory("HTO7A");
        public static Trajectory T7AToH = new Trajectory("7AToH");
        public static Trajectory THTO7B = new Trajectory("HTO7B");

    
        
        public static void update(){
            for(Trajectory t : trajectories){
                t.update();
            }
        }
        
    }

}
