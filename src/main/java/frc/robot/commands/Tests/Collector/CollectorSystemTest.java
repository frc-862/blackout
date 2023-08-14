package frc.robot.commands.Tests.Collector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.thunder.command.core.TimedCommand;

public class CollectorSystemTest extends SequentialCommandGroup {

	public CollectorSystemTest(Collector collector, double maxPower, double maxRPM) {
		super(new WaitCommand(2),
			new TimedCommand(new CollectorRPMTest(collector, maxRPM), 5),
			new WaitCommand(1),
			new TimedCommand(new CollectorTest(collector, maxPower), 5));
	}
}
