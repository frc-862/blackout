package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.FalconConfig;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The collector subsystem
 */
public class Collector extends SubsystemBase {
    // The collector motor
    private TalonFX insideMotor;
    private TalonFX outsideMotor;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    private int currSupplyCurrentLimit = CollectorConstants.SUPPLY_CURRENT_LIMIT;
    private int currStatorCurrentLimit = CollectorConstants.STATOR_CURRENT_LIMIT;

    private GamePiece gamePiece = GamePiece.CUBE;

    public Collector() {
        // Create the motor and configure it
        insideMotor = FalconConfig.createMotor(CAN.INSIDE_COLLECTOR_MOTOR,
                CollectorConstants.INSIDE_MOTOR_INVERT, CollectorConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, CollectorConstants.NEUTRAL_MODE);
        outsideMotor = FalconConfig.createMotor(CAN.OUTSIDE_COLLECTOR_MOTOR,
                CollectorConstants.OUTSIDE_MOTOR_INVERT, CollectorConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, CollectorConstants.NEUTRAL_MODE);

        // Initialize the shuffleboard values and start logging data
        initialiizeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Sets the power of the collector motors
     * 
     * @param power the percent speed to set the collector motors to
     */
    public void setPower(double power) {
        insideMotor.set(TalonFXControlMode.PercentOutput, power);
        outsideMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets the power of the Inside collector motor
     * 
     * @param power the percent speed to set the inside collector motor to
     */
    public void setInsidePower(double power) {
        insideMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets the power of the Outside collector motor
     * 
     * @param power the percent speed to set the outside collector motor to
     */
    public void setOutsidePower(double power) {
        outsideMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    // Method to start logging
    @SuppressWarnings("unchecked")
    private void initialiizeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Collector", CollectorConstants.LOG_PERIOD,
            new Pair<String, Object>("Collector motor output percent", (DoubleSupplier) () -> insideMotor.getMotorOutputPercent()));
    }

    /**
     * Sets supply current limit if its different from the current supply current limit
     * 
     * @param supplyCurrentLimit the new smart current limit
     */
    public void setSupplyCurrentLimit(int supplyCurrentLimit) {
        if (supplyCurrentLimit != currSupplyCurrentLimit) {
            insideMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    supplyCurrentLimit, supplyCurrentLimit, .25), 250);
            outsideMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    supplyCurrentLimit, supplyCurrentLimit, .25), 250);
            // TODO Threshold needs testing, tigger threshold timer (s) , Timeout for application (ms)
        }
        currStatorCurrentLimit = supplyCurrentLimit;
    }

    /**
     * Sets stator current limit if its different from the current stator current limit
     * 
     * @param statorLimit
     */
    public void setStatorCurrentLimit(int statorLimit) {
        if (statorLimit != currStatorCurrentLimit) {
            insideMotor.configStatorCurrentLimit(
                    new StatorCurrentLimitConfiguration(true, statorLimit, statorLimit, .25), 250);
            outsideMotor.configStatorCurrentLimit(
                    new StatorCurrentLimitConfiguration(true, statorLimit, statorLimit, .25), 250);
        }
        currStatorCurrentLimit = statorLimit;
    }


    /**
     * Returns true if the collector is stalling
     */
    public boolean isStalling() {
        return (insideMotor.getStatorCurrent() > CollectorConstants.STALL_POWER)
            || (outsideMotor.getStatorCurrent() > CollectorConstants.STALL_POWER); // TODO Test Amount and type Stator vs Supply
    }

    /**
     * Gets the game piece detected by the color sensor
     * 
     * @return the game piece detected by the color sensor (Either CUBE, CONE, or NONE)
     */
    public GamePiece getGamePiece() {
        return gamePiece;
    }

    // For the driver to set the game piece manually
    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    /**
     * stop Sets the power of the collector motor to 0
     */
    public void stop() {
        setPower(0d);
    }

    @Override
    public void periodic() {

        periodicShuffleboard.loop();
    }
}
