package frc.robot.commands.Tests.Collector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.thunder.command.core.TimedCommand;

public class CollectorSystemTest extends SequentialCommandGroup {

	public CollectorSystemTest(Collector collector, double maxPower) {
		super(new WaitCommand(2),
			new TimedCommand(new CollectorInsideTest(collector, maxPower), 2),
			new WaitCommand(1),
			new TimedCommand(new CollectorOutsideTest(collector, maxPower), 2),
			new WaitCommand(1),
			new TimedCommand(new CollectorTest(collector, maxPower), 2));
	}
}
