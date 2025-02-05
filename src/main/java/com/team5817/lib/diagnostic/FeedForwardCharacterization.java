package com.team5817.lib.diagnostic;

import edu.wpi.first.wpilibj.DriverStation;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.List;

import com.team5817.frc2025.autos.Actions.Action;
import com.team5817.lib.PolynomialRegression;
import com.team5817.lib.drivers.ServoMotorSubsystem;

public class FeedForwardCharacterization implements Action{
  private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.05;

  private final boolean forwards;

  private final ServoMotorSubsystem mSubsystem;


  private final FeedForwardCharacterizationData dataPrimary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a differential drive. */
  /** Creates a new FeedForwardCharacterization for a simple subsystem. */

  public FeedForwardCharacterization(
      ServoMotorSubsystem subsystem,
      boolean forwards,
      FeedForwardCharacterizationData data) {
        this.mSubsystem = subsystem;
        this.forwards = forwards;
        this.dataPrimary = data;
  }

  // Called when the command is initially scheduled.
  @Override
  public void start() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void update() {
    if (timer.get() < startDelaySecs) {
        mSubsystem.applyVoltage(0);
   } else {
      double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
        mSubsystem.applyVoltage(voltage);
      dataPrimary.add(mSubsystem.getVelocity(), voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void done() {
    mSubsystem.applyVoltage(0);
    timer.stop();
    dataPrimary.print();
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }

  public static class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      System.out.println("FF Characterization Results (" + name + "):");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
    }
  }




}