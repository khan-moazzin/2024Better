package com.team5817.frc2025.autos.Actions;


public class LambdaAction implements Action {

	public interface VoidInterace {
		void function();
	}

	VoidInterace mFunction;

	public LambdaAction(VoidInterace function) {
		this.mFunction = function;
	}

	@Override
	public void start() {
		mFunction.function();
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {}
}
