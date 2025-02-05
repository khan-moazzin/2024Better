package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.ControlsCheck;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitforControllerInput;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Indexer.Indexer;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.Intake.IntakeRollers;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.Elastic;
import com.team5817.lib.Elastic.Notification;
import com.team5817.lib.Elastic.Notification.NotificationLevel;

public class TestRoutine extends AutoBase {
    
    
    private final double delayStart = 2.0;

	private Elevator mElevator;
	private EndEffectorWrist mEndEffectorWrist;
	private IntakeDeploy mIntakeDeploy;
	private Climb mClimb;
	private EndEffectorRollers mEndEffectorRollers;
	private IntakeRollers mIntakeRollers;
	private Indexer mIndexer;
    private Superstructure s;


    public TestRoutine(){
        s = Superstructure.getInstance();
        mElevator = Elevator.getInstance();
        mEndEffectorWrist = EndEffectorWrist.getInstance();
        mIntakeDeploy = IntakeDeploy.getInstance();
        mClimb = Climb.getInstance();
        mEndEffectorRollers = EndEffectorRollers.getInstance();
        mIntakeRollers = IntakeRollers.getInstance();
        mIndexer = Indexer.getInstance();
    }

    public void routine(){

        System.out.println("Starting Test Routine");
        r(new WaitAction(delayStart));
        
        mElevator.stateRequest(Elevator.State.L4).act();
        System.out.println("Elevator Up");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Elevator Up"));
        r(new WaitAction(1));
        r(new WaitforControllerInput());
        mElevator.stateRequest(Elevator.State.ZERO).act();
        System.out.println("Elevator should be at ZERO");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Elevator should be at ZERO"));
        r(new WaitAction(1));
        r(new WaitforControllerInput());
        r(new WaitAction(1));

        mEndEffectorWrist.stateRequest(EndEffectorWrist.State.INTAKING).act();
        System.out.println("Wrist INTAKING");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Wrist INTAKING"));
        r(new WaitforControllerInput());
        mEndEffectorWrist.stateRequest(EndEffectorWrist.State.ZERO).act();
        System.out.println("Wrist should be at ZERO");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Wrist should be at ZERO"));
        r(new WaitforControllerInput());
        r(new WaitAction(1));

        mIntakeDeploy.stateRequest(IntakeDeploy.State.DEPLOY).act();
        System.out.println("Intake DEPLOYING");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Intake DEPLOYING"));
        r(new WaitforControllerInput());
        mIntakeDeploy.stateRequest(IntakeDeploy.State.ZERO).act();
        System.out.println("Intake should be at ZERO");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Intake should be at ZERO"));
        r(new WaitforControllerInput());
        r(new WaitAction(1));

        mClimb.stateRequest(Climb.State.PULL).act();
        System.out.println("Climb PULLING");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Climb PULLING"));
        r(new WaitforControllerInput());
        mClimb.stateRequest(Climb.State.ZERO).act();
        System.out.println("Climb should be at Zero");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Climb should be at ZERO"));
        r(new WaitforControllerInput());
        r(new WaitAction(1));
        
        mEndEffectorWrist.stateRequest(EndEffectorWrist.State.INTAKING).act();
        mEndEffectorRollers.stateRequest(EndEffectorRollers.State.CORAL_INTAKE).act();
        System.out.println("Manipulator should be INTAKING Coral");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Manipulator should be INTAKING Coral"));
        r(new WaitforControllerInput());
        mEndEffectorRollers.stateRequest(EndEffectorRollers.State.CORAL_OUTTAKE).act();
        mEndEffectorWrist.stateRequest(EndEffectorWrist.State.ZERO).act();
        System.out.println("Manipulator should be OUTTAKING Coral");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Manipulator should be OUTTAKING Coral"));
        r(new WaitforControllerInput());
        mEndEffectorRollers.stateRequest(EndEffectorRollers.State.IDLE).act();
        r(new WaitAction(1));

        mIntakeDeploy.stateRequest(IntakeDeploy.State.DEPLOY).act();
        mIntakeRollers.setState(IntakeRollers.State.INTAKING_CORAL);
        System.out.println("Intake should Be INTAKING");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Intake should be INTAKING"));
        r(new WaitforControllerInput());
        mIntakeRollers.setState(IntakeRollers.State.IDLE);
        mIntakeDeploy.stateRequest(IntakeDeploy.State.STOW).act();
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Intake STOWED"));
        System.out.println("Intake STOWED");
        r(new WaitAction(1));
        r(new WaitforControllerInput());
        r(new WaitAction(1));

        mIndexer.stateRequest(Indexer.State.INDEXING).act();
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Indexer should be INDEXING"));
        System.out.println("Indexer should be INDEXING");
        r(new WaitforControllerInput());
        mIndexer.stateRequest(Indexer.State.IDLE).act();
        r(new WaitAction(1));

        System.out.println("Entering Teleop Check");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Entering Teleop Check"));

        r(new ControlsCheck());
        r(new WaitAction(1));
        s.setGoal(GoalState.STOW);       

        System.out.println("Test Routine Finished");
        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "TEST ROUTINE", "Test Routine Finished"));

    }
}
