package com.team5817.frc2024.subsystems;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends Subsystem{
    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    IntakeOutputsAutoLogged outputs = new IntakeOutputsAutoLogged();

    TalonFX mIntakeMotor = new TalonFX(0);
    @Override
    public void readPeriodicInputs() {
        inputs.intakeHasUpdate = mIntakeMotor.getSupplyCurrent().getValueAsDouble()>20;
        Logger.processInputs("Intake", inputs);
    }
    @Override 
    public void writePeriodicOutputs() {
        mIntakeMotor.set(outputs.speed);
    }
    public void setIntakeSpeed(double speed){
        outputs.speed = speed;
    }
    @AutoLog
    public static class IntakeInputs{
        public boolean intakeHasUpdate;
    }
    @AutoLog
    public static class IntakeOutputs{
        public double speed = 0;
    }
}
