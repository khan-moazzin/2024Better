// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2025;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.BuildConstants;
import com.team5817.frc2025.controlboard.ControlBoard;
import com.team5817.frc2025.controlboard.DriverControls;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.loops.Looper;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.vision.VisionDeviceManager;
import com.team5817.lib.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends LoggedRobot {
    public static Pose3d[] mechPoses = new Pose3d[6];
    static{
        for(int i = 0; i < 6; i++){
            mechPoses[i] = new Pose3d();
        }

    }
    SubsystemManager mSubsystemManager;
    // Superstructure s;
    VisionDeviceManager mVision = VisionDeviceManager.getInstance();
    DriverControls controls;
    ControlBoard controlBoard = ControlBoard.getInstance();
    // public LoggedDashboardChooser<AutoBase> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    private final Looper mEnabledLooper = new Looper();

    SwerveDriveSimulation drivesim;
    Drive drive;
  // HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
    @SuppressWarnings("resource")
    @Override
    public void robotInit() {
      DriverStation.silenceJoystickConnectionWarning(true);
  
  
      DriverStation.startDataLog(DataLogManager.getLog());
  
      RobotState.getInstance().reset();
      // for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
        // String N = entry.getKey();
        // AutoBase A = entry.getValue();
        // autoChooser.addOption(N, A);
      // }
  
      Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

if (isReal()) {
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
} else {
    if(Constants.mode == Constants.Mode.REPLAY) {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      setUseTiming(false);
    }else{
      drivesim = new SwerveDriveSimulation(Drive.mapleSimConfig,new Pose2d(3,3,Rotation2d.identity()).wpi());
      Drive.registerDriveSimulation(drivesim);
      SimulatedArena.getInstance().addDriveTrainSimulation(drivesim);
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging  
    }
  }

Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  // s = Superstructure.getInstance();
          
      drive = Drive.getInstance();
      mSubsystemManager = SubsystemManager.getInstance();

      controls = new DriverControls();
      mSubsystemManager.setSubsystems(
          Drive.getInstance(),
          Superstructure.getInstance(),
          VisionDeviceManager.getInstance(),
          IntakeDeploy.getInstance(),
          Climb.getInstance(),
          Elevator.getInstance(),
          EndEffectorRollers.getInstance(),
          EndEffectorWrist.getInstance()
          );
          
          mSubsystemManager.registerEnabledLoops(mEnabledLooper);
          mEnabledLooper.start();
       }
  
    @Override
    public void robotPeriodic() {
      // auto = autoChooser.get();
      
      // mEnabledLooper.outputToSmartDashboard();
      //     mSubsystemManager.outputLoopTimes();
      Logger.recordOutput("Mechs", mechPoses);
      
    }
  
  
boolean disableGyroReset = false;
    @Override
    public void autonomousInit() {
      disableGyroReset = true;
      drive.resetModulesToAbsolute();
      // Superstructure.getInstance().setState(Superstructure.AUTO);
      // autoExecuter.setAuto(auto);
      // autoExecuter.start();
    }
  
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() { 
    }
  
    /** This function is called once when teleop is enabled. */  
    @Override
    public void teleopInit() {
      // swerve.fieldzeroSwerve();
      drive.resetModulesToAbsolute();
      drive.feedTeleopSetpoint(new ChassisSpeeds(0,0,0));
    }
  
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
      controls.twoControllerMode();
      // controls.oneControllerMode();
      controlBoard.update();
      if(!Robot.isReal() && Constants.mode == Constants.Mode.SIM){
        drive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
          controlBoard.getSwerveTranslation().x(),
          controlBoard.getSwerveTranslation().y(),
          controlBoard.getSwerveRotation(),
          Rotation2d.kIdentity

      ));
      }else{
        drive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
          controlBoard.getSwerveTranslation().x(),
          controlBoard.getSwerveTranslation().y(),
          controlBoard.getSwerveRotation(),
            Util.robotToFieldRelative(drive.getHeading(), DriverStation.getAlliance().get().equals(Alliance.Red))
      ));}


    }

    
  
    /** This function is called once when the robot is disabled. */
  
    @Override
    public void disabledInit() {
      mSubsystemManager.stop();
      // Superstructure.getInstance().clearQueues();
      // autoExecuter.stop();
      // autoExecuter = new AutoExecuter();
    }
  
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
      RobotState.getInstance().outputTelemetry();
      // if(!disableGyroReset)
      // drive.zeroGyro(mVision.getMovingAverageRead());
    }
  
    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }
  
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", drivesim.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
  }
 