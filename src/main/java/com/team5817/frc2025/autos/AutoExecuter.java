package com.team5817.frc2025.autos;


public class AutoExecuter{

    private Thread mThread = null;

    private AutoBase mAuto = null;

    private boolean mActive;


    public void setAuto(AutoBase auto){
        this.mAuto = auto;
    }

    public AutoBase getAuto(){
        return this.mAuto;
    }

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

   public void stop(){
        if(mAuto != null){
            mAuto.stop();
        }
        mActive = false;
        mThread = null;

   }

} 