package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Collector;

/**
 * Command for running the collector with a triggers
 */
public class Collect extends CommandBase {
    Collector collector;
    DoubleSupplier input;

    /**
     * Creates a new Collect command
     * 
     * @param collector the collector subsystem
     * @param input the input speed for the collector
     */
    public Collect(Collector collector, DoubleSupplier input) {
        this.collector = collector;
        this.input = input;

        addRequirements(collector);
    }

    @Override
    public void execute() { // TODO See How this works with new Design
        if (collector.getGamePiece() == GamePiece.CONE) {
            collector.setPower(-input.getAsDouble());
        } else {
            collector.setPower(input.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
