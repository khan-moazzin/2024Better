package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.autos.AutoBase;

public class DoNothingMode extends AutoBase{

	public DoNothingMode() {
	}

	// spotless:off
	@Override
	public void routine() {
        System.out.println("Did Nothing");
        System.out.println("Finished auto!");
	}
	// spotless:on
   
}
