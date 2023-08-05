package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.FalconConfig;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;
import frc.thunder.shuffleboard.LightningShuffleboard;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

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
                CollectorConstants.STATOR_CURRENT_LIMIT, CollectorConstants.NEUTRAL_MODE,
                CollectorConstants.INSIDE_kP, CollectorConstants.INSIDE_kI, 
                CollectorConstants.INSIDE_kD);
        outsideMotor = FalconConfig.createMotor(CAN.OUTSIDE_COLLECTOR_MOTOR,
                CollectorConstants.OUTSIDE_MOTOR_INVERT, CollectorConstants.SUPPLY_CURRENT_LIMIT,
                CollectorConstants.STATOR_CURRENT_LIMIT, CollectorConstants.NEUTRAL_MODE,
                CollectorConstants.OUTSIDE_kP, CollectorConstants.OUTSIDE_kI, 
                CollectorConstants.OUTSIDE_kD);

        // Initialize the shuffleboard values and start logging data
        // initialiizeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Sets the power of the collector motors
     * -1.0 <> 1.0
     * @param power the percent speed to set the collector motors to
     */
    public void setPercentPower(double power) {
        insideMotor.set(power);
        outsideMotor.set(power);
    }

    /**
     * Sets the power of the collector motors in RPM
     * 
     * @param RPM the velocity to set the collector motors to in RPM
     */
    public void setRPM(double RPM) {
        double RPS = RPM / 60; // TODO check

        insideMotor.setControl(new VelocityVoltage(RPS, true, CollectorConstants.INSIDE_FF, 0, false));
        outsideMotor.setControl(new VelocityVoltage(RPS, true, CollectorConstants.OUTSIDE_FF, 0, false));
    }

    /**
     * Sets the power of the Inside collector motor
     * -1.0 <> 1.0
     * @param power the percent speed to set the inside collector motor to
     */
    public void setInsidePower(double power) {
        insideMotor.set(power);
    }

    /**
     * Sets the power of the Outside collector motor
     * -1.0 <> 1.0
     * @param power the percent speed to set the outside collector motor to
     */
    public void setOutsidePower(double power) {
        outsideMotor.set(power);
    }

    // // Method to start logging
    // @SuppressWarnings("unchecked")
    // private void initialiizeShuffleboard() {
    //     periodicShuffleboard = new LightningShuffleboardPeriodic("Collector", CollectorConstants.LOG_PERIOD,
    //         new Pair<String, Object>("Collector motor output percent", (DoubleSupplier) () -> insideMotor.getMotorOutputPercent()));
    // }

    /**
     * Sets supply current limit if its different from the current supply current limit
     * 
     * @param supplyCurrentLimit the new smart current limit
     */
    public void setSupplyCurrentLimit(int supplyCurrentLimit) {
        if (supplyCurrentLimit != currSupplyCurrentLimit) {
            // insideMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
            //         supplyCurrentLimit, supplyCurrentLimit, .25), 10);
            // outsideMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
            //         supplyCurrentLimit, supplyCurrentLimit, .25), 10);
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
            // insideMotor.configStatorCurrentLimit(
            //         new StatorCurrentLimitConfiguration(true, statorLimit, statorLimit, .25), 250);
            // outsideMotor.configStatorCurrentLimit(
            //         new StatorCurrentLimitConfiguration(true, statorLimit, statorLimit, .25), 250);
        }
        currStatorCurrentLimit = statorLimit;
    }


    /**
     * Returns true if the collector is stalling
     */
    public boolean isStalling() {
        return ( insideMotor.getTorqueCurrent().getValue() > CollectorConstants.STALL_POWER)
            || (outsideMotor.getTorqueCurrent().getValue() > CollectorConstants.STALL_POWER); // TODO Test Amount and type Stator vs torque
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

    public void setCoastMode() { //TODO see if possible probably not nessecary
        // insideMotor.setNeutralMode(NeutralModeValue.Coast);
        // outsideMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBrakeMode() { //TODO see if possible probably not nessecary
        // insideMotor.setNeutralMode(NeutralModeValue.Brake);
        // outsideMotor.setNeutralModeValue(NeutralModeValue.Brake);
    }

    // public double currentEncoderTicks(TalonFX motor) {
	// 	// return motor.getSelectedSensorPosition();
	// }

    // public double getCurrentRPM(TalonFX motor) {
	// 	return motor.getSelectedSensorVelocity() / 2048 * 600; //converts from revs per second to revs per minute
	// }

    /**
     * gets current RPM from internal encoders
     * @return average RPM of collector
     */
    public double getCurrentRPM() {
        // Starts in RPS
		return ((insideMotor.getVelocity().getValue() * 60) + (outsideMotor.getVelocity().getValue() * 60)) / 2; //converts from revs per second to revs per minute
	}

    /**
     * stop Sets the power of the collector motor to 0
     */
    public void stop() {
        insideMotor.stopMotor();
        outsideMotor.stopMotor();
    }

    @Override
    public void periodic() {

        periodicShuffleboard.loop();

        // insideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "INSIDE_kP", CollectorConstants.INSIDE_kP));
        // insideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "INSIDE_kI", CollectorConstants.INSIDE_kI));
        // insideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "INSIDE_kD", CollectorConstants.INSIDE_kD));
        // insideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "INSIDE_kF", CollectorConstants.INSIDE_kF));

        // outsideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "OUTSIDE_kP", CollectorConstants.INSIDE_kP));
        // outsideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "OUTSIDE_kI", CollectorConstants.INSIDE_kI));
        // outsideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "OUTSIDE_kD", CollectorConstants.INSIDE_kD));
        // outsideMotor.config_kP(0, LightningShuffleboard.getDouble("Collector", "OUTSIDE_kF", CollectorConstants.INSIDE_kF));

        LightningShuffleboard.setDouble("Collector", "RPM", getCurrentRPM());

        // LightningShuffleboard.setDouble("Collector", "INSIDE_Ticks", currentEncoderTicks(insideMotor));
        // LightningShuffleboard.setDouble("Collector", "OUTSIDE_Ticks", currentEncoderTicks(outsideMotor));

        setRPM(LightningShuffleboard.getDouble("Collector", "Set RPM", 0));
    }
}
