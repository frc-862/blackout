package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.WristAngles.wristStates;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Wrist;


public class HoldPower extends CommandBase {
    Collector collector;
    DoubleSupplier input;
    boolean doHoldPower = false;
    double power = 0;
    XboxController driver;
    XboxController copilot;
    Wrist wrist;

    /**
     * Creates a new Collect command
     * 
     * @param collector the collector subsystem
     * @param input the input speed for the collector
     * @param driver Drivers Controller
     * @param copilot Copilot Controller
     * @param Wrist The wrist subsystem
     */
    public HoldPower(Collector collector, DoubleSupplier input, XboxController driver,
            XboxController copilot, Wrist wrist) {
        this.collector = collector;
        this.input = input;
        this.driver = driver;
        this.copilot = copilot;
        this.wrist = wrist;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (input.getAsDouble() > 0) { // Collector collects
            wrist.setGoalState(wristStates.Ground);
            doHoldPower = true;
            power = input.getAsDouble();
        } else if (input.getAsDouble() < 0) { // Collector spits
            doHoldPower = false;
            // wrist.setGoalState(wristStates.Ground);
            power = input.getAsDouble();
        } else if (doHoldPower) { // Hold power if no input and last input was inwards
            wrist.setGoalState(wristStates.Stow);
            power = CollectorConstants.HOLD_POWER;
        } else {
            wrist.setGoalState(wristStates.Stow);
            power = 0;
        }

        // if (input.getAsDouble() < 0) {
        // collector.setCurrentLimit(60);
        // } else {
        // collector.setCurrentLimit(CollectorConstants.CURRENT_LIMIT); // TODO Check if correct
        // Current Limit
        // }

        if (DriverStation.isTeleop()) {
            if (collector.isStalling()) { // For Drivers to know when the piece is in
                driver.setRumble(RumbleType.kBothRumble, 1);
                copilot.setRumble(RumbleType.kBothRumble, 1);
            } else {
                driver.setRumble(RumbleType.kBothRumble, 0);
                copilot.setRumble(RumbleType.kBothRumble, 0);
            }

            collector.setPercentPower(power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
