package frc.robot.commands.Tests.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectorTest extends CommandBase {

	Collector collector;

	double maxPower;
	double power = 0d;

	public CollectorTest(Collector collector, double maxPower) {
		this.collector = collector;
		this.maxPower = maxPower;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPercentPower(0d);
		collector.setCoastMode();
	}

	@Override
	public void execute() {
		if (power < maxPower) {
			power += 0.05d;
		}
		collector.setPercentPower(power);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		collector.setPercentPower(0d);
		collector.setBrakeMode();
	}

}
