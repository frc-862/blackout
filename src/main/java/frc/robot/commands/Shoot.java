package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristAngles;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Wrist;

public class Shoot extends CommandBase {
	
	Collector collector;
	Wrist wrist;
	double targetRPM = 0;
	
	public Shoot(Collector collector, Wrist wrist) {
		this.collector = collector;
		this.wrist = wrist;

		addRequirements(collector);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if(wrist.getGoalState() == wrist.getCurrState()) {
			if(wrist.getCurrState() == wristStates.Stow) {
				//TODO what to do when we are stowed
			} else {
				targetRPM = WristAngles.shootSpeedMap().get(wrist.getCurrState());
			}
		}
	}

	@Override
	public void execute() {
		collector.setRPM(targetRPM);
	}

	@Override
	public void end(boolean interrupted) {
		collector.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false; // TODO determine when to end
		// TIME based, RPM, AMPS?
	}
}
