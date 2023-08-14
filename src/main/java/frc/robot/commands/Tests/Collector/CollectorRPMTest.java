package frc.robot.commands.Tests.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectorRPMTest extends CommandBase {

	Collector collector;
	double maxRPM;
	double RPM = 0d;

	public CollectorRPMTest(Collector collector, double maxRPM) {
		this.collector = collector;
		this.maxRPM = maxRPM;

		addRequirements(collector);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		collector.stop();
		collector.setCoastMode();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RPM <= maxRPM) {
			RPM += 50d;
		}
		collector.setRPM(RPM);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		collector.stop();
		collector.setBrakeMode();
	}

	@Override
	public boolean isFinished(){
		return RPM >= maxRPM;
	}
}
