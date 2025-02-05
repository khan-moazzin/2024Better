package com.team5817.frc2025.autos.Actions;

import java.lang.ModuleLayer.Controller;

import com.team5817.frc2025.controlboard.CustomXboxController;

public class WaitforControllerInput implements Action{

    CustomXboxController mController;


    @Override
    public void start() {
        mController = new CustomXboxController(0);
    }

    @Override
    public void update() {
        mController.update();
    }

    @Override
    public void done() {
        mController = null;
    }

    @Override
    public boolean isFinished() {
        return mController.getAButtonPressed();
    }

}
    