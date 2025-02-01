package com.team5817.frc2025.autos;


import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.team5817.frc2025.autos.Actions.Action;

import edu.wpi.first.wpilibj.Timer;



public abstract class AutoBase{

    protected SwerveDriveSimulation mSim;
    private double startTime;

    boolean mActive = false;
    public void start() {
        mActive = true;
        startTime = Timer.getTimestamp();
        routine();
        end();
   }

   public void end(){
        System.out.println("Auto Ended in " + (Timer.getTimestamp() - startTime) + " seconds");
   }

	public abstract void routine();

    public void stop() {
        mActive = false;
    }

    public void r(Action action){
        action.start();
        if(!mActive)
            throw new Error("Action ran while auto is inactive");
        while(mActive && !action.isFinished()){
            action.update();
            try{
                Thread.sleep(20);
            }
            catch(InterruptedException e){
                e.printStackTrace();
            }
        } 
        if(!mActive)
            throw new Error("Action Inturupted");
        action.done();
    }


    public void registerDriveSimulation(SwerveDriveSimulation sim){
        mSim = sim;
    }








}