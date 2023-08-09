package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.WristAngles;
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
		collector.setPercentPower(-CollectorConstants.HOLD_POWER_CUBE);;
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
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return collector.getCurrentRPM() >= WristAngles.shootSpeedMap().get(wrist.getCurrState());
	}
}
