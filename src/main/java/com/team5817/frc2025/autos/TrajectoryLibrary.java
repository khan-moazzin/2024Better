package com.team5817.frc2025.autos;

import java.io.IOException;
import java.nio.file.DirectoryIteratorException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.stream.Stream;

import com.team5817.lib.motion.Trajectory;

/**
 * A library of predefined trajectories for the robot.
 */
public class TrajectoryLibrary {

    /**
     * A nested class containing static trajectory instances and a method to update them.
     */
    public static class l {
        public static HashMap<String, Trajectory> trajectories = new HashMap<>();

        public static void init(){
            Path folderPath;

        // Check if running on RoboRIO or on a local machine (Simulation)
        if (Files.exists(Paths.get("/home/lvuser/deploy/pathplanner/paths"))) {
            folderPath = Paths.get("/home/lvuser/deploy/pathplanner/paths"); // RoboRIO path
        } else {
            folderPath = Paths.get("src/main/deploy/pathplanner/paths"); // Local simulation path
        }
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(folderPath)) {
            for (Path file : stream) {
                if (Files.isRegularFile(file)) {
                    String name = file.getFileName().toString();
                    name = name.substring(0, name.length()-5);
                    new Trajectory(name);
                }
            }
        } catch (IOException | DirectoryIteratorException e) {
            e.printStackTrace();
        }
    }
        /**
         * Updates all trajectories in the library.
         */
        public static void update(){
            for(Trajectory t : trajectories.values()){
                t.update();
            }
        }
        
    }

}
