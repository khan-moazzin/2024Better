package com.team5817.frc2025.autos;

/**
 * The AutoExecuter class is responsible for managing the execution of autonomous routines.
 */
public class AutoExecuter{

    private Thread mThread = null;
    private AutoBase mAuto = null;
    private boolean mActive;

    /**
     * Sets the autonomous routine to be executed.
     * 
     * @param auto The autonomous routine to be set.
     */
    public void setAuto(AutoBase auto){
        this.mAuto = auto;
    }

    /**
     * Gets the currently set autonomous routine.
     * 
     * @return The currently set autonomous routine.
     */
    public AutoBase getAuto(){
        return this.mAuto;
    }

    /**
     * Starts the execution of the autonomous routine in a new thread.
     */
    public void start(){
        mActive = true;
        if(mThread == null){
            mThread = new Thread(new Runnable(){
                @Override
                public void run(){
                    if(mActive){
                        if(mAuto != null)
                            mAuto.start();   
                    }
                }
            });
            mThread.start();
        }
    }

    /**
     * Stops the execution of the autonomous routine and resets the thread.
     */
    public void stop(){
        if(mAuto != null){
            mAuto.stop();
        }
        mActive = false;
        mThread = null;
    }
}