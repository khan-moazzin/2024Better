package com.team5817.frc2025.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.team5817.lib.motion.Trajectory;

public class TrajectoryLibrary {

    public static class l {
        public static HashMap<String, Trajectory> trajectories = new HashMap<>();



        public static Trajectory TCTo3A = new Trajectory("CTo_3A");
        public static Trajectory TCTo8A = new Trajectory("CTo_8A");
        public static Trajectory TCTo8B = new Trajectory("CTo_8B");

        public static Trajectory TSTo3A = new Trajectory("STo_3A");
        public static Trajectory TSTo8A = new Trajectory("STo_8A");
        public static Trajectory TSTo8B = new Trajectory("STo_8B");
        
        public static Trajectory T3AToFH = new Trajectory("_3AToFH");
        public static Trajectory T6BToFH = new Trajectory("_6BToFH");
        public static Trajectory T7AToFH = new Trajectory("_7AToFH");
        public static Trajectory T7BToFH = new Trajectory("_7BToFH");
        public static Trajectory T8AToFH = new Trajectory("_8AToFH");
        public static Trajectory T8BToFH = new Trajectory("_8BToFH");

        public static Trajectory T3AToCH = new Trajectory("_3AToCH");
        public static Trajectory T6BToCH = new Trajectory("_6BToCH");
        public static Trajectory T7AToCH = new Trajectory("_7AToCH");
        public static Trajectory T7BToCH = new Trajectory("_7BToCH");
        public static Trajectory T8AToCH = new Trajectory("_8AToCH");
        public static Trajectory T8BToCH = new Trajectory("_8BToCH");


        public static Trajectory TFHTo3A = new Trajectory("FHTo_3A");
        public static Trajectory TFHTo6B = new Trajectory("FHTo_6B");
        public static Trajectory TFHTo7A = new Trajectory("FHTo_7A");
        public static Trajectory TFHTo7B = new Trajectory("FHTo_7B");
        public static Trajectory TFHTo8A = new Trajectory("FHTo_8A");
        public static Trajectory TFHTo8B = new Trajectory("FHTo_8B");

        public static Trajectory TCHTo3A = new Trajectory("CHTo_3A");
        public static Trajectory TCHTo6B = new Trajectory("CHTo_6B");
        public static Trajectory TCHTo7A = new Trajectory("CHTo_7A");
        public static Trajectory TCHTo7B = new Trajectory("CHTo_7B");
        public static Trajectory TCHTo8A = new Trajectory("CHTo_8A");
        public static Trajectory TCHTo8B = new Trajectory("CHTo_8B");




        public static void update(){
            for(Trajectory t : trajectories.values()){
                t.update();
            }
        }
        
    }

}
