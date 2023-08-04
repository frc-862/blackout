package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.subsystems.Wrist;

public class wristState extends CommandBase {
	
	Wrist wrist;

	wristStates currState;
	wristStates goalState;

	public wristState(Wrist wrist) {
		this.wrist = wrist;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		goalState = wristStates.stow;
		currState = wristStates.stow;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (currState != goalState) {

		}
	}

	public void setGoalState(wristStates goalState) {
		this.goalState = goalState;
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}
}
