// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2025.controls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.team5817.lib.Util;


public class CustomXboxController extends XboxController {
  private double DEAD_BAND = 0.15;
  private static final double PRESS_THRESHOLD = 0.05;

  private static CustomXboxController instance = null;

  public static CustomXboxController getInstance() {
    if (instance == null) {
      instance = new CustomXboxController(0);
    }
    return instance;
  }


  public final ButtonCheck leftStickX = new ButtonCheck();
  public final ButtonCheck leftStickY = new ButtonCheck();
  public final ButtonCheck rightStickX = new ButtonCheck();
  public final ButtonCheck rightStickY = new ButtonCheck();
  public final ButtonCheck leftTrigger = new ButtonCheck(0.5);
  public final ButtonCheck rightTrigger = new ButtonCheck(0.5);
  public final ButtonCheck leftBumper = new ButtonCheck();
  public final ButtonCheck rightBumper = new ButtonCheck();
  public final ButtonCheck leftStick = new ButtonCheck();
  public final ButtonCheck rightStick = new ButtonCheck();
  public final ButtonCheck aButton = new ButtonCheck();
  public final ButtonCheck bButton = new ButtonCheck();
  public final ButtonCheck xButton = new ButtonCheck();
  public final ButtonCheck yButton = new ButtonCheck();
  public final ButtonCheck startButton = new ButtonCheck();
  public final ButtonCheck backButton = new ButtonCheck();
  public final ButtonCheck dpadUp = new ButtonCheck();
  public final ButtonCheck dpadDown = new ButtonCheck();
  public final ButtonCheck dpadLeft = new ButtonCheck();
  public final ButtonCheck dpadRight = new ButtonCheck();

  private boolean lastRumble = false;
  private final Timer rumbleTimer = new Timer();
  private double rumbleStart = Double.NEGATIVE_INFINITY;

  public CustomXboxController(int port) {
	super(port); 
	rumbleTimer.start();
  }

  public void rumble(boolean rumble) {
    if (rumble != lastRumble && rumble) {
      rumbleStart = rumbleTimer.get();
    }

    if (rumbleTimer.get() - rumbleStart < 1.0) {
      this.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      this.setRumble(RumbleType.kBothRumble, 0.0);
    }

    lastRumble = rumble;
  }

  public void update() {
	leftStickX.update(this.getLeftX());
	leftStickY.update(this.getLeftY());
	rightStickX.update(this.getRightX());
	rightStickY.update(this.getRightY());
	leftTrigger.update(this.getLeftTriggerAxis());
	rightTrigger.update(this.getRightTriggerAxis());
	leftBumper.update(this.getLeftBumper());
	rightBumper.update(this.getRightBumper());
	leftStick.update(this.getLeftStickButton());
	rightStick.update(this.getRightStickButton());
	aButton.update(this.getAButton());
	bButton.update(this.getBButton());
	xButton.update(this.getXButton());
	yButton.update(this.getYButton());
	startButton.update(this.getStartButton());
	backButton.update(this.getBackButton());

    int pov = this.getPOV();
    dpadUp.update(pov == 0);
    dpadRight.update(pov == 90);
    dpadDown.update(pov == 180);
    dpadLeft.update(pov == 270);
  }

  public void setDeadband(double deadband) {
	DEAD_BAND = deadband;
	
  }






  @Override
  public double getLeftX() {
	  return Util.deadBand(getRawAxis(0), DEAD_BAND);
  }

  @Override
  public double getRightX() {
	  return Util.deadBand(getRawAxis(4), DEAD_BAND);
  }

  @Override
  public double getLeftY() {
	  return Util.deadBand(getRawAxis(1), DEAD_BAND);
  }

  @Override
  public double getRightY() {
	  return Util.deadBand(getRawAxis(5), DEAD_BAND);
  }

  @Override
  public double getLeftTriggerAxis() {
	  return Util.deadBand(getRawAxis(2), PRESS_THRESHOLD);
  }

  @Override
  public double getRightTriggerAxis() {
	  return Util.deadBand(getRawAxis(3), PRESS_THRESHOLD);
  }



  public static class ButtonCheck {
    private double value = 0.0;
    private double threshold;
    private boolean current = false;
    private boolean previous = false;
	boolean buttonActive = false;
	boolean activationReported = false;



    public ButtonCheck(double threshold) {
      this.threshold = threshold;
    }

    public ButtonCheck() {
      this(0.0);
    }

    public void update(double val) {
      value = val;
    }

    public void update(boolean state) {
      previous = current;
      current = state;
    }

    public boolean isPressed() {
      return current && !previous;
    }

    public boolean isReleased() {
      return !current && previous;
    }

    public boolean isActive() {
      return current;
    }

	public boolean isBeingPressed() {
		return buttonActive;
	}

	public boolean wasActivated() {
		if (buttonActive && !activationReported) {
			activationReported = true;
			return true;
		}
		return false;
	
}

    public double getValue() {
      return Math.abs(value) >= threshold ? value : 0.0;
    }

    public void setThreshold(double threshold) {
      this.threshold = threshold;
    }
  }
}
