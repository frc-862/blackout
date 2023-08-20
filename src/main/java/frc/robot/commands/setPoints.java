package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Wrist;

public class setPoints extends CommandBase {

	Wrist wrist;
	double targetPower = 0;
	// wristStates targetState;

	Supplier<wristStates> targetState;

	public setPoints(Wrist wrist, Supplier<wristStates> targetState) {
		this.wrist = wrist;
		this.targetState = targetState;

		addRequirements(wrist);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		wrist.setGoalState(targetState.get());
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
		wrist.setGoalState(wristStates.Stow);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
