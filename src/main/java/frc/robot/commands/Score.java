package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.WristAngles;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Wrist;

public class Score extends CommandBase {
	
	Collector collector;
	Wrist wrist;
	double targetRPM = 0;
	// wristStates targetState;

	Supplier<wristStates> targetState;
	
	public Score(Collector collector, Wrist wrist, Supplier<wristStates> targetState) {
		this.collector = collector;
		this.wrist = wrist;
		this.targetState = targetState;

		addRequirements(collector);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		collector.setPercentPower(-CollectorConstants.HOLD_POWER_CUBE);
		wrist.setGoalState(targetState.get());
	}

	@Override
	public void execute() {
		if(wrist.getGoalState() == wrist.getCurrState() && wrist.onTarget()) {
			targetRPM = WristAngles.shootSpeedMap().get(wrist.getCurrState());
			collector.setRPM(targetRPM);
		}
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
		wrist.setGoalState(wristStates.Stow);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return collector.getCurrentRPM() >= WristAngles.shootSpeedMap().get(wrist.getCurrState());
		return false;
	}
}
