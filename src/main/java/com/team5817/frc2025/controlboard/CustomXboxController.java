// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2025.controlboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class CustomXboxController {
  private final XboxController controller;

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
    controller = new XboxController(port);
    rumbleTimer.start();
  }

  public void rumble(boolean rumble) {
    if (rumble != lastRumble && rumble) {
      rumbleStart = rumbleTimer.get();
    }

    if (rumbleTimer.get() - rumbleStart < 1.0) {
      controller.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

    lastRumble = rumble;
  }

  public void update() {
    leftStickX.update(controller.getLeftX());
    leftStickY.update(controller.getLeftY());
    rightStickX.update(controller.getRightX());
    rightStickY.update(controller.getRightY());
    leftTrigger.update(controller.getLeftTriggerAxis());
    rightTrigger.update(controller.getRightTriggerAxis());
    leftBumper.update(controller.getLeftBumper());
    rightBumper.update(controller.getRightBumper());
    leftStick.update(controller.getLeftStickButton());
    rightStick.update(controller.getRightStickButton());
    aButton.update(controller.getAButton());
    bButton.update(controller.getBButton());
    xButton.update(controller.getXButton());
    yButton.update(controller.getYButton());
    startButton.update(controller.getStartButton());
    backButton.update(controller.getBackButton());

    int pov = controller.getPOV();
    dpadUp.update(pov == 0);
    dpadRight.update(pov == 90);
    dpadDown.update(pov == 180);
    dpadLeft.update(pov == 270);
  }

  public static class ButtonCheck {
    private double value = 0.0;
    private double threshold;
    private boolean current = false;
    private boolean previous = false;

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

    public double getValue() {
      return Math.abs(value) >= threshold ? value : 0.0;
    }

    public void setThreshold(double threshold) {
      this.threshold = threshold;
    }
  }
}
