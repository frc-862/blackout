package frc.robot.commands.Tests.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectorInsideTest extends CommandBase {

	Collector collector;
	double maxpower;
	double power = 0d;

	public CollectorInsideTest(Collector collector, double maxpower) {
		this.collector = collector;
		this.maxpower = maxpower;

		addRequirements(collector);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		collector.setPower(0d);
    	collector.setCoastMode();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(power < maxpower) {
			power += 0.05d;
		}
		collector.setInsidePower(power);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		collector.setPower(0d);
    	collector.setBrakeMode();
	}
}
